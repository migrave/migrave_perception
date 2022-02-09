/*
 * Author: Mohammad Wasil
 * Based on the example from OpenPose project
 * https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/14_synchronous_custom_input.cpp
 */
#ifndef MIGRAVE_SKELETON_TRACKING_OPENPOSE_HELPER_HPP
#define MIGRAVE_SKELETON_TRACKING_OPENPOSE_HELPER_HPP

#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

namespace openpose
{
void configureOPWrapper(op::Wrapper* opWrapper) 
{
  try
  {
    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
                  "Wrong logging_level value.",
                  __LINE__,
                  __FUNCTION__,
                  __FILE__);

    // Applying user defined configuration - GFlags to program variables
    // Set input configs to default
    op::ProducerType producerType = op::ProducerType::None;
    op::String producerString = "";

    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    // Applying user defined configuration
    // Set cameraSize to 640x480 (Asus and Intel realsense defaul res)
    op::String image_size = "640x480";
    const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution),
                                             image_size);

    // outputSize, defaul example is "-1x-1"
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), 
                                             image_size);

    // netInputSize
    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), 
                                               "-1x256");

    // faceNetInputSize, default example is 368x368
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), 
                                                   "256x256 (multiples of 16)");

    // handNetInputSize,default example is 368x368
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), 
                                                   "256x256 (multiples of 16)");

    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);

    // poseModel
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));

    // JSON saving
    // if (!FLAGS_write_keypoint.empty())
    //     std::cout<<"Flag write_keypoint is deprecated and will eventually be removed. 
    //                 Please, use write_json instead.i"<<std::endl;

    // keypointScaleMode
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);

    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                  FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);

    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);

    // >1 camera view?
    // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    const auto multipleView = false;

    // Face and hand detectors
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);

    // Enabling Google Logging
    const bool enableGoogleLogging = true;

    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{poseMode,
                                                  netInputSize,
                                                  // FLAGS_net_resolution_dynamic,
                                                  outputSize,
                                                  keypointScaleMode,
                                                  FLAGS_num_gpu,
                                                  FLAGS_num_gpu_start,
                                                  FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose,
                                                                        multipleView),
                                                  poseModel,
                                                  !FLAGS_disable_blending,
                                                  (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap,
                                                  FLAGS_part_to_show,
                                                  op::String(FLAGS_model_folder),
                                                  heatMapTypes,
                                                  heatMapScaleMode,
                                                  FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold,
                                                  FLAGS_number_people_max,
                                                  FLAGS_maximize_positives,
                                                  FLAGS_fps_max,
                                                  op::String(FLAGS_prototxt_path),
                                                  op::String(FLAGS_caffemodel_path),
                                                  (float)FLAGS_upsampling_ratio,
                                                  enableGoogleLogging};
    opWrapper->configure(wrapperStructPose);

    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face,
                                                  faceDetector,
                                                  faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose,
                                                  (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    opWrapper->configure(wrapperStructFace);

    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand,
                                                  handDetector,
                                                  handNetInputSize,
                                                  FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range,
                                                  op::flagsToRenderMode(FLAGS_hand_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose,
                                                  (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};
    opWrapper->configure(wrapperStructHand);

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    // WrapperStructExtra allows the user to set up the pose estimation and rendering 
    // parameters that will be used for. Eg. Whether to enable people tracking across 
    // frames; Person tracking id; Performing 3-D reconstruction from the multiple views.
    const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d,
                                                    FLAGS_3d_min_views,
                                                    FLAGS_identification,
                                                    FLAGS_tracking,
                                                    FLAGS_ik_threads};
    opWrapper->configure(wrapperStructExtra);

    // Producer (use default to disable any input)
    const op::WrapperStructInput wrapperStructInput{
        producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
        FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
        cameraSize, op::String(FLAGS_camera_parameter_path), FLAGS_frame_undistort, FLAGS_3d_views};
    opWrapper->configure(wrapperStructInput);             
    
    // Output (comment or use default argument to disable any output)
    const op::WrapperStructOutput wrapperStructOutput{};
    opWrapper->configure(wrapperStructOutput);

    // GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display,
                                                                       FLAGS_3d),
                                                !FLAGS_no_gui_verbose,
                                                FLAGS_fullscreen};
    opWrapper->configure(wrapperStructGui);

    // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
    if (FLAGS_disable_multi_thread) opWrapper->disableMultiThreading();
  }
  catch (const std::exception& e)
  {
    std::cout<<"Error "<< e.what(); 
    std::cout<<"at line number "<< __LINE__;
    std::cout<<" on function "<<__FUNCTION__<< "in file "<< __FILE__ << std::endl;
  }
}

} // namespace openpose

#endif  // MIGRAVE_SKELETON_TRACKING_OPENPOSE_HELPER_HPP
