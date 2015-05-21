using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using AForge;
using AForge.Video.VFW;
using System.Drawing;
using System.Drawing.Imaging;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    public class VideoStreamSaver
    {

        private int width;
        private int height;
        private AVIWriter writer = null;

        public VideoStreamSaver(string filename, int width, int height){
            this.width = width;
            this.height = height;
            writer = new AVIWriter();
            writer.Open(filename, width,height);
        }

        public void addFrame(Bitmap newFrame) {
            this.writer.AddFrame(newFrame);
        }
        public void close() {
            this.writer.Close();
        }
    }
}
