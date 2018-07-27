./bonnet_model_compiler.par \
    --frozen_graph_path=frozen_inference_graph.pb \
    --output_graph_path=coco_object_detection.binaryproto \
    --input_tensor_name="Preprocessor/sub" \
    --output_tensor_names="concat,concat_1" \
    --input_tensor_size=256