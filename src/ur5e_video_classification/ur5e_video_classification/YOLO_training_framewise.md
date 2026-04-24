# Classificació de Vídeos amb YOLO — Documentació del Pipeline

## Visió general

El pipeline transforma una col·lecció de vídeos etiquetats per classe en un model de classificació d'imatges basat en **YOLO**. L'enfocament és **frame-level**: en lloc de modelar seqüències temporals, s'extreuen fotogrames representatius de cada vídeo i el model aprèn a classificar imatges estàtiques individuals. La predicció final sobre un vídeo s'obté per votació majoritària entre els seus frames.

Aquest enfocament és deliberadament senzill — no hi ha modelat temporal ni arquitectures recurrents — i funciona bé quan les classes es poden distingir visualment a partir d'un sol fotograma.

El pipeline consta de dues etapes principals:

1. `prepare_frames` — extracció i organització dels frames com a dataset d'imatges.
2. `train_yolo` — entrenament del classificador YOLO sobre aquell dataset.

---

## Dependències

```bash
pip install opencv-python numpy tqdm ultralytics pyyaml
```

| Paquet | Versió mínima | Ús |
|---|---|---|
| `opencv-python` | 4.x | Lectura i decodificació de vídeos |
| `ultralytics` | 8.x | Framework YOLO |
| `numpy` | 1.x | Mostreig d'índexs de frames |
| `tqdm` | — | Barres de progrés |
| `pyyaml` | — | Generació del fitxer de configuració |

---

## Estructura de dades

### Vídeos d'entrada

Els vídeos s'han d'organitzar per classe i, **dins de cada classe, per split** (train/val). Separar els vídeos per split a nivell de carpeta evita qualsevol risc de contaminació entre conjunts — cap frame d'un vídeo de validació pot acabar barrejat amb frames d'entrenament.

```
videos/
├── classe_A/
│   ├── train/
│   │   ├── video1.mp4
│   │   ├── video2.mp4
│   │   └── ...
│   └── val/
│       ├── video3.mp4
│       └── ...
├── classe_B/
│   ├── train/
│   │   └── ...
│   └── val/
│       └── ...
└── classe_C/
    ├── train/
    │   └── ...
    └── val/
        └── ...
```

Cada subdirectori de primer nivell és una classe; el seu nom serà l'etiqueta del model. La proporció recomanada és **80% train / 20% val**, assignant vídeos sencers a cada split (no frames individuals).

### Dataset de frames generat

`prepare_frames` produeix la següent estructura, que és el format estàndard que espera YOLO per a classificació:

```
yolo_frames/
├── train/
│   ├── classe_A/
│   │   ├── video1_frame_000.jpg
│   │   ├── video1_frame_001.jpg
│   │   └── ...
│   └── classe_B/
│       └── ...
└── val/
    ├── classe_A/
    │   └── ...
    └── classe_B/
        └── ...
```

---

## Mòdul 1: `prepare_frames`

### `extract_frames_from_video`

Extreu `N` fotogrames distribuïts uniformement al llarg d'un vídeo. Usa `np.linspace` per garantir que els índexs siguin equidistants independentment de la durada del vídeo.

```python
import cv2
import numpy as np

def extract_frames_from_video(video_path, num_frames=16):
    """
    Args:
        video_path (str | Path): Ruta al fitxer de vídeo.
        num_frames (int): Nombre de frames a extreure.

    Returns:
        list[np.ndarray]: Frames en format BGR.
    """
    cap = cv2.VideoCapture(str(video_path))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if total_frames == 0:
        print(f"[WARN] No s'ha pogut llegir: {video_path}")
        cap.release()
        return []

    if total_frames >= num_frames:
        frame_indices = np.linspace(0, total_frames - 1, num_frames, dtype=int)
    else:
        frame_indices = np.arange(total_frames)

    frames = []
    for idx in frame_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if ret:
            frames.append(frame)

    cap.release()
    return frames
```

### `prepare_dataset`

Itera per l'estructura de directoris `classe/split/`, extreu frames de cada vídeo i els desa redimensionats al directori de sortida. La separació train/val ja ve donada per l'estructura de carpetes d'entrada — no es fa cap shuffling ni divisió aleatòria en aquest punt.

```python
from pathlib import Path
from tqdm import tqdm

def prepare_dataset(video_dir, output_dir, frames_per_video=16, frame_size=640):
    """
    Args:
        video_dir (str | Path): Directori arrel amb estructura classe/split/.
        output_dir (str | Path): On es guardarà el dataset de frames.
        frames_per_video (int): Frames a extreure per vídeo.
        frame_size (int): Mida en píxels (quadrada) dels frames de sortida.
    """
    video_dir = Path(video_dir)
    output_dir = Path(output_dir)
    video_extensions = ['.mp4', '.avi', '.mov', '.mkv', '.webm']

    class_dirs = [d for d in video_dir.iterdir() if d.is_dir()]
    print(f"Classes trobades: {[d.name for d in class_dirs]}")

    for class_dir in class_dirs:
        class_name = class_dir.name

        for split in ['train', 'val']:
            split_input_dir = class_dir / split
            split_output_dir = output_dir / split / class_name
            split_output_dir.mkdir(parents=True, exist_ok=True)

            if not split_input_dir.exists():
                print(f"[WARN] No existeix: {split_input_dir}, saltant...")
                continue

            videos = []
            for ext in video_extensions:
                videos.extend(list(split_input_dir.glob(f'*{ext}')))

            if not videos:
                print(f"[WARN] Cap vídeo a {split_input_dir}")
                continue

            print(f"\n{class_name}/{split}: {len(videos)} vídeos")

            for video_path in tqdm(videos, desc=f"  {class_name}/{split}"):
                frames = extract_frames_from_video(video_path, frames_per_video)
                video_stem = video_path.stem

                for i, frame in enumerate(frames):
                    frame_resized = cv2.resize(frame, (frame_size, frame_size))
                    filename = f"{video_stem}_frame_{i:03d}.jpg"
                    cv2.imwrite(str(split_output_dir / filename), frame_resized)

    print(f"\nDataset guardat a: {output_dir}")
```

### Paràmetres

| Paràmetre | Valor per defecte | Notes |
|---|---|---|
| `frames_per_video` | `16` | Augmentar si els vídeos tenen molta variabilitat interna |
| `frame_size` | `640` | Mida estàndard per a YOLO; usar el mateix valor a l'entrenament |

---

## Mòdul 2: `train_yolo`

### `create_yolo_config`

YOLO necessita un fitxer `.yaml` que descrigui la localització del dataset i el mapeig d'índex a nom de classe. Les classes s'infereixen automàticament dels subdirectoris de `train/`.

```python
import yaml
from pathlib import Path

def create_yolo_config(data_dir, output_path='yolo_data.yaml'):
    """
    Args:
        data_dir (str | Path): Directori arrel del dataset (conté train/ i val/).
        output_path (str): Ruta on es desarà el fitxer YAML.

    Returns:
        tuple[str, list[str]]: (ruta_config, llista_de_classes)
    """
    data_dir = Path(data_dir)
    class_names = sorted([
        d.name for d in (data_dir / 'train').iterdir() if d.is_dir()
    ])

    config = {
        'path': str(data_dir.absolute()),
        'train': 'train',
        'val': 'val',
        'names': {i: name for i, name in enumerate(class_names)}
    }

    with open(output_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"Config generat: {output_path} — classes: {class_names}")
    return output_path, class_names
```

Exemple del fitxer generat:

```yaml
path: /ruta/absoluta/yolo_frames
train: train
val: val
names:
  0: classe_A
  1: classe_B
  2: classe_C
```

### `train_yolo_classifier`

Carrega un model YOLO preentrenat i fa fine-tuning sobre el dataset de frames. Els pesos del millor epoch (millor accuracy de validació) es desen automàticament a `weights/best.pt`.

```python
from ultralytics import YOLO

def train_yolo_classifier(
    data_config,
    model_name='yolo11n-cls.pt',
    epochs=50,
    batch_size=32,
    imgsz=640,
    project='yolo_classifier',
    name='run'
):
    """
    Args:
        data_config (str): Ruta al fitxer YAML de configuració.
        model_name (str): Checkpoint de partida (vegeu taula de models).
        epochs (int): Nombre màxim d'èpoques.
        batch_size (int): Mida del batch.
        imgsz (int): Mida d'entrada de les imatges; ha de coincidir amb frame_size.
        project (str): Directori on es guarden els resultats.
        name (str): Nom de l'experiment (subdirectori dins de project).
    """
    model = YOLO(model_name)

    model.train(
        data=data_config,
        epochs=epochs,
        batch=batch_size,
        imgsz=imgsz,
        project=project,
        name=name,
        patience=20,  # early stopping, si no hi ha millores en x epoches el model para d'entrenar
        save=True,
        plots=True
    )

    model.val()
    print(f"\nMillor model: {project}/{name}/weights/best.pt")
```

### Models disponibles

| Model | Mida | Paràmetres aprox. | Ús recomanat |
|---|---|---|---|
| `yolo11n-cls.pt` | Nano | ~1.5M | Prototipat ràpid, pocs recursos |
| `yolo11s-cls.pt` | Small | ~6M | Balanç velocitat/precisió |
| `yolo11m-cls.pt` | Medium | ~20M | Producció amb GPU |
| `yolo11l-cls.pt` | Large | ~43M | Màxima precisió |

---

## Mòdul 3: Inferència

La predicció sobre un nou vídeo segueix el mateix procés d'extracció de frames. La classe final s'obté per **votació majoritària**: cada frame emet un vot per a la seva classe predita, i guanya la classe amb més vots.

```python
from collections import Counter
from ultralytics import YOLO
import numpy as np

def predict_video(video_path, model_path, num_frames=16):
    """
    Args:
        video_path (str): Ruta al vídeo a classificar.
        model_path (str): Ruta al model entrenat (best.pt).
        num_frames (int): Nombre de frames a analitzar.

    Returns:
        dict: {
            'class': classe predita,
            'confidence': confiança mitjana per a la classe guanyadora,
            'votes': distribució de vots per classe,
            'frame_predictions': llista de (classe, confiança) per frame
        }
    """
    model = YOLO(model_path)
    frames = extract_frames_from_video(video_path, num_frames)

    if not frames:
        return None

    predictions = []
    for frame in frames:
        result = model.predict(frame, verbose=False)[0]
        top1_class = result.names[result.probs.top1]
        top1_conf  = float(result.probs.top1conf)
        predictions.append((top1_class, top1_conf))

    votes = Counter(p[0] for p in predictions)
    winner = votes.most_common(1)[0][0]
    avg_conf = np.mean([p[1] for p in predictions if p[0] == winner])

    return {
        'class': winner,
        'confidence': avg_conf,
        'votes': dict(votes),
        'frame_predictions': predictions
    }
```

---

## Execució del pipeline complet

```python
# 1. Extreure frames i construir el dataset
prepare_dataset(
    video_dir='./videos',       # estructura classe/train|val/
    output_dir='./yolo_frames',
    frames_per_video=16,
    frame_size=640
)

# 2. Generar la configuració de YOLO
config_path, classes = create_yolo_config('./yolo_frames')

# 3. Entrenar el model
train_yolo_classifier(
    data_config=config_path,
    model_name='yolo11n-cls.pt',
    epochs=50,
    batch_size=32,
    imgsz=640,
    project='yolo_classifier',
    name='run_01'
)

# 4. Inferència sobre un vídeo nou
result = predict_video(
    video_path='nou_video.mp4',
    model_path='yolo_classifier/run_01/weights/best.pt'
)
print(f"{result['class']}  ({result['confidence']:.1%})  — vots: {result['votes']}")
```

---

## Sortides de l'entrenament

YOLO desa tots els artefactes de l'experiment a `{project}/{name}/`:

```
yolo_classifier/run_01/
├── weights/
│   ├── best.pt            ← pesos del millor epoch (usar per inferència)
│   └── last.pt            ← pesos de l'últim epoch
├── results.png            ← corbes de loss i accuracy
├── confusion_matrix.png   ← matriu de confusió sobre val
└── val_batch0_pred.jpg    ← mostra visual de prediccions
```