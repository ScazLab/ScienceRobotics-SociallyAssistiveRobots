This is what should be completed on the first day:
1. sample the child and parent's face and train the model
2. assess the tracking system with the samples

To sample the face, roslaunch (every time you run this file all previous training will lose):
tracking_initializer_single_face.launch

Please have only face in the camera at one time. There will be instructions on the screen of what should be doing.
Click on the camera view to start sampling the faces, and click again will stop sampling. Mom first and then dad and then child.
Each person will be taken samples from different angles:
- look forward
- look left
- look right
- look up
- look down
- head lean to left
- head lean to right
- look at top left corner of the screen
- look at top right corner of the screen
- look at bottom left corner of the screen
- look at bottom right corner of the screen
- look at robot
- look at each other
- look at non-targets (not screen, not robot, not each other)

Afterwards click on the camera view will train the model. Collecting about 30 samples (the number will show while collecting sample) of each angle should suffice. Too many samples will make the training slow.

Two models will be trained, one is child and mom, and the other is child and dad. It is normal for the screen to grey out or stop responding while training the model.

Afterwards it is the single face assessment. This only evaluate the tracking and identification of the face when only one face present. So it doesn't matter where the person is looking at. But please do not let them have too extreme of gestures such as facing the left too much that the system cannot track the face. In the assessment is also child first and then mom and then dad.

After the assessment the program will still running, with the predict shown. Click on screen will switch between mom's model and dad's model. But you can ctrl-c to terminate the program, or take a look at what the current prediction is to get an idea.

-----------------------------------------------------------------------------------------------------------------------------------
If you are not satisfied with the sample or the prediction result, there are several things you can do:

1. delete some samples and re-train the sample
The sample pictures are stored in the subfolder /home/sar/face_analyzer_data
Then to train the sample, roslaunch:
tracking_initializer_single_face_train_predict.launch
This will start at the training stage and then the steps are the same as in tracking_initializer_single_face.launch (e.g., it will do the training and then assessment and then only do predict).

2. If you just want to do the assessment using the model trained, roslaunch:
tracking_initializer_single_face_assessment.launch

3. If you just want to take a look at the live prediction without doing any tasks like training or assessmeng, roslaunch:
tracking_initializer_single_face_predict.launch
-----------------------------------------------------------------------------------------------------------------------------------

And then you need to do assessment on both face.
To assess child and dad, do this
roslaunch: tracking_initializer_both_face_assessment_dad.launch
To assess child and mom, do this
roslaunch: tracking_initializer_both_face_assessment_mom.launch

Also click on the camera view to start assessing. It will assess identification when both face present. And then let them look at screen, look at robot, look at each other, and look at other in sequence to assess targets. There will be instructions on the screen of what to do next.
