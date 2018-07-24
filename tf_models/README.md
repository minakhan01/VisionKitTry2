# Notes on TensorFlow frozen graphs

File Name                                      | input tensor                               | output tensors                    | (height, width) | (input_mean, input_stddev)
:--------------------------------------------- | :----------------------------------------- | :-------------------------------- | :-------------- | :-------------------------
`mobilenet_v1_160res_0.5_imagenet.pb`          | 'input'                                    | 'MobilenetV1/Predictions/Softmax' | (160, 160)      | (128.0, 128.0)
`mobilenet_ssd_256res_0.125_person_cat_dog.pb` | 'Preprocessor/sub'                         | 'concat', 'concat_1'              | (256, 256)      | (128.0, 128.0)
`squeezenet_160res_5x5_0.75.pb`                | 'input'                                    | 'Prediction'                      | (160, 160)      | (128.0, 128.0)
`mobilenet_v1_192res_1.0_seefood.pb`           | 'input'                                    | 'MobilenetV1/Predictions/Softmax' | (192, 192)      | (128.0, 128.0)
`mobilenet_v2_192res_1.0_plant.pb`             | 'map/TensorArrayStack/TensorArrayGatherV3' | 'prediction'                      | (192, 192)      | (128.0, 128.0)
`mobilenet_v2_192res_1.0_insect.pb`            | 'map/TensorArrayStack/TensorArrayGatherV3' | 'prediction'                      | (192, 192)      | (128.0, 128.0)
`mobilenet_v2_192res_1.0_bird.pb`              | 'map/TensorArrayStack/TensorArrayGatherV3' | 'prediction'                      | (192, 192)      | (128.0, 128.0)
