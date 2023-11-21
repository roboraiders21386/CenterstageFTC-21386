package org.firstinspires.ftc.teamcode.OpModes.autonomous.OpModes.autonomous;

import org.openftc.easyopencv.OpenCvCamera;

public class MyCameraOpenListener implements OpenCvCamera.AsyncCameraOpenListener {
    @Override
    public void onOpened() {
        // Implement your code to handle camera opening here
    }

    @Override
    public void onError(int errorCode) {
        // Implement your code to handle any errors that occur during camera opening here
    }
}
