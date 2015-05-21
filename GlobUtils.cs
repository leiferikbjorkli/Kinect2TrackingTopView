using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    static class GlobUtils
    {

        static public int getIndex(int x, int y)
        {
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                throw new System.ArgumentException("Coordinates are incorrect");
            }

            return y * GlobVar.scaledFrameWidth + x;
        }

        static public Point getPoint(int index)
        {
            if (index < 0 || index > GlobVar.scaledFrameLength-1)
            {
                throw new System.ArgumentException("Index out of bounds");
            }
            Point p;
            p.x = index % GlobVar.scaledFrameWidth;
            double yDouble = index / GlobVar.scaledFrameWidth;
            int y = (int)Math.Floor(yDouble);
            p.y = y;
            return p;
        }

        static public int calculatePixelAreaFromIndexes(int a, int b, int c)
        {
            return (getPoint(b).x + 1 - getPoint(a).x) * (getPoint(c).y + 1 - getPoint(a).y);
        }
    }
}
