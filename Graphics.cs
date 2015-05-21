using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Windows;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    class Graphics
    {
        public static void DrawRectangle(byte[] image, Rectangle rect)
        {
            for (int i = 0; i < rect.width; i++)
            {
                image[GlobUtils.getIndex(rect.xStart + i, rect.yStart)] = (byte)255;
                image[GlobUtils.getIndex(rect.xStart + i, rect.yStart + rect.height)] = (byte)255;
            }
            for (int j = 0; j < rect.height; j++)
            {
                image[GlobUtils.getIndex(rect.xStart, rect.yStart + j)] = (byte)255;
                image[GlobUtils.getIndex(rect.xStart + rect.width, rect.yStart + j)] = (byte)255;
            }
        }

        public static void DrawRectangle(byte[] image, ThreePointRectangle rect )
        {
            int topLength = rect.b.x - rect.a.x;
            int sideLength = rect.c.y - rect.a.y;

            for (int i = 0; i < topLength; i++)
            {
                image[GlobUtils.getIndex(rect.a.x + i, rect.a.y)] = (byte)255;
                image[GlobUtils.getIndex(rect.c.x + i, rect.c.y)] = (byte)255;
            }
            for (int i = 0; i < sideLength; i++)
            {
                image[GlobUtils.getIndex(rect.a.x, rect.a.y + i)] = (byte)255;
                image[GlobUtils.getIndex(rect.b.x, rect.b.y + i)] = (byte)255;
            }
        }
        public static void DrawRectangle(byte[] image, IndexRectangle rect)
        {
            ThreePointRectangle pointRect = new ThreePointRectangle(GlobUtils.getPoint(rect.a), GlobUtils.getPoint(rect.b), GlobUtils.getPoint(rect.c));
            DrawRectangle(image, pointRect);
        }


        public static void DrawPoint(byte[] image, int x, int y)
        {
            image[GlobUtils.getIndex(x, y)] = (byte)255;
        }

    }
}
