using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Drawing.Imaging;
using AForge.Vision.Motion;

namespace InteractionDetection
{
    public class MotionTracker
    {


        public Bitmap track(Bitmap newImage) {
            MotionDetector detector = new MotionDetector(new CustomFrameDifferenceDetector(), new MotionAreaHighlighting());
            detector.ProcessFrame(newImage);


            return newImage;
        }
    }
}
