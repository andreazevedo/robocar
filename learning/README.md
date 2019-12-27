# Model Training

This section contains utilities for TensorFlow model training.

## Installing pre-requisits

### 1) Install tensorflow
Go to [tensorflow](https://www.tensorflow.org/install/pip?lang=python3) website
and follow the install instructions.

### 2) Install tensorflow models
```bash
# Clone tensorflow/models
$ git clone git@github.com:tensorflow/models.git

# Update PYTHONPATH
$ cd models/research
$ export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim

# Build object_detector protos
$ cd object_detector
$ protoc object_detection/protos/*.proto --python_out=./
```

### 3) Install LabelImg
Go to the [labelImg](https://github.com/tzutalin/labelImg) website and follow
the install intructions.

### 4) Install other pre-requisits
```bash
$ pip3 install pandas matplotlib numpy cython pycocotools
```

## Training Step-by-step

### 1) Get images
Download some images (or take some pictures) and save them in `data/images`.
There are already some images about stop signs in this repo that can be used
for training.
If you are going to train an existing model, you should have around 200 images
for training and another 60 images for validation.

### 2) Label the images
Use [labelImg](https://github.com/tzutalin/labelImg) to label images. LabelImg
is an awesome free tool that can be used to easily label images.

When using LabelImg, go to `Open dir` and select the `data/images` directory.
Then click on `Change Save Dir` and select `data/annotations`.

### 3) Separate annotations
Separate your annotations into `train` and `validate` directories.
About 30% of your images should be used for validation. For example, if you have
260 images in total, move 200 annotations to `data/annotations/train` and about
60 annotations to `data/annotations/validate`.

### 4) Convert the labels to the TFRecord format
```bash
$ python3 xml_to_csv.py -in data/annotations/train/ -out data/train.csv

$ python3 xml_to_csv.py -in data/annotations/validate/ -out data/validate.csv

$ python3 generate_tfrecord.py --input_csv=data/train.csv \
  --output_tfrecord=data/train.record

$ python3 generate_tfrecord.py --input_csv=data/validate.csv \
  --output_tfrecord=data/validate.record
```

### 5) Update config
Update `data/models/ssd_mobilenet_v2_quantized.config` with your path.

### 6) Train the model
```bash
$ cd PATH_TO_TENSORFLOW_MODELS/research/

$ python3 object_detection/model_main.py \
    --pipeline_config_path=[PIPELINE_CONFIG_PATH] \
    --model_dir=[MODEL_DIR] \
    --num_train_steps=[NUM_TRAIN_STEPS] \
    --sample_1_of_n_eval_examples=1 \
    --alsologtostderr
```

### 7) Export frozen graph for tflite
```bash
$ cd PATH_TO_TENSORFLOW_MODELS/research/

$ python3 object_detection/export_inference_graph.py \
    --pipeline_config_path=[PIPELINE_CONFIG_PATH] \
    --trained_checkpoint_prefix=[PATH_TO_CHECKPOINT_WITHOUT_.INDEX]
    --output_directory=[PATH_TO_SAVE_EXPORTED_MODEL]
    --add_postprocessing_op=true
```

### 8) Export to tflite
```bash
$ cd PATH_TO_TENSORFLOW

$ bazel run -c opt tensorflow/lite/toco:toco -- \
    --input_file=[PATH_TO_EXPORED_MODEL]/frozen_inference_graph.pb \
    --output_file=[PATH_TO_TFLITE_MODEL] \
    --input_shapes=1,300,300,3 \
    --input_arrays=normalized_input_image_tensor \
    --output_arrays='TFLite_Detection_PostProcess','TFLite_Detection_PostProcess:1','TFLite_Detection_PostProcess:2','TFLite_Detection_PostProcess:3'  \
    --inference_type=QUANTIZED_UINT8 \
    --mean_values=128 \
    --std_values=128 \
    --change_concat_input_ranges=false \
    --allow_custom_ops
```

## References

 - https://github.com/tensorflow/models/tree/master/research/object_detection
 - https://3sidedcube.com/guide-retraining-object-detection-models-tensorflow/
 - https://github.com/datitran/raccoon_dataset
