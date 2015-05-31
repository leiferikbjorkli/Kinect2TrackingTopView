using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Windows;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class Graphics
    {
        public static void DrawRectangle(Rectangle rect)
        {


            int xStart = GlobUtils.GetPoint(rect.IndexStart).x;
            int yStart = GlobUtils.GetPoint(rect.IndexStart).y;

            if (GlobUtils.BoundaryCheck(xStart,yStart))
            {
                xStart = GlobUtils.AdjustXBoundaries(xStart);
                yStart = GlobUtils.AdjustXBoundaries(yStart);
            }

            int xStop = GlobUtils.GetPoint(rect.IndexStart).x + rect.Width;
            int yStop = GlobUtils.GetPoint(rect.IndexStart).y + rect.Height;

            if (GlobUtils.BoundaryCheck(xStop,yStop))
            {
                xStop = GlobUtils.AdjustXBoundaries(xStop);
                yStop = GlobUtils.AdjustXBoundaries(yStop);
            }

            for (int i = 0; i < rect.Width; i++)
            {
                GlobVar.canvas[GlobUtils.GetIndex(xStart + i, yStart)] = (byte)255;
                GlobVar.canvas[GlobUtils.GetIndex(xStart + i, yStart + rect.Height)] = (byte)255;
            }
            for (int j = 0; j < rect.Height; j++)
            {
                GlobVar.canvas[GlobUtils.GetIndex(xStart, yStart + j)] = (byte)255;
                GlobVar.canvas[GlobUtils.GetIndex(xStart + rect.Width, yStart + j)] = (byte)255;
            }
        }

        public static void DrawRectangle(ThreePointRectangle rect )
        {
            int topLength = rect.b.x - rect.a.x;
            int sideLength = rect.c.y - rect.a.y;

            for (int i = 0; i < topLength; i++)
            {
                GlobVar.canvas[GlobUtils.GetIndex(rect.a.x + i, rect.a.y)] = (byte)255;
                GlobVar.canvas[GlobUtils.GetIndex(rect.c.x + i, rect.c.y)] = (byte)255;
            }
            for (int i = 0; i < sideLength; i++)
            {
                GlobVar.canvas[GlobUtils.GetIndex(rect.a.x, rect.a.y + i)] = (byte)255;
                GlobVar.canvas[GlobUtils.GetIndex(rect.b.x, rect.b.y + i)] = (byte)255;
            }
        }
        public static void DrawRectangle(IndexRectangle rect)
        {
            ThreePointRectangle pointRect = new ThreePointRectangle(GlobUtils.GetPoint(rect.a), GlobUtils.GetPoint(rect.b), GlobUtils.GetPoint(rect.c));
            DrawRectangle(pointRect);
        }


        public static void DrawPoint(int x, int y)
        {
            GlobVar.canvas[GlobUtils.GetIndex(x, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y - 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y - 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x, y - 1)] = (byte)255;
        }
        public static void DrawPoint(Point point)
        {
            int x = point.x;
            int y = point.y;

            GlobVar.canvas[GlobUtils.GetIndex(x, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y - 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x + 1, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y - 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x - 1, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x, y + 1)] = (byte)255;
            GlobVar.canvas[GlobUtils.GetIndex(x, y - 1)] = (byte)255;
        }

        public static void DrawCanvas(byte[] intensityMap)
        {
            for (int i = 0; i < intensityMap.Length; i++)
            {
                if (GlobVar.canvas[i] != 0)
                {
                    intensityMap[i] = GlobVar.canvas[i];
                }
            }
        }

        public static void ClearCanvas()
        {
            for (int i = 0; i < GlobVar.scaledFrameLength; i++)
            {
                GlobVar.canvas[i] = (byte)0;
            }
        }

        public static void DrawBodies()
        {

            int t = GlobVar.canvas.Max();
            
            foreach (var head in GlobVar.Heads)
            {
                foreach (var pixel in head.HeadPixels)
                {
                    if (GlobVar.canvas[pixel] == 0)
                    {
                        GlobVar.canvas[pixel] = (byte) head.Label;

                    }
                }
            }
        }

        public static void DrawLine(float slope, float intercept,float r)
        {

            float start = -(float) (GlobVar.MaxHorizontalWidth/1000);
            float end = -start;


            for (float i = start; i < end; i+=0.01f)
            {
                float x = i;
                float y = x*slope + intercept;

                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.scaledFrameWidth / 2) + (GlobVar.scaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.scaledFrameHeight / 2) + (GlobVar.scaledFrameHeight / 2));

                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);

                if (index != -1)
                {
                    GlobVar.canvas[index] = 255;
                }
            }
        }

        public static void DrawPoint(float x, float y)
        {

            // Can compute factor offline

            int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.scaledFrameWidth / 2) + (GlobVar.scaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.scaledFrameHeight / 2) + (GlobVar.scaledFrameHeight / 2));

            DrawPoint(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
        }

        public static void DrawLineNoPointcloud(float slope, float intercept,float r)
        {
            float start = -(float) (GlobVar.MaxHorizontalWidth/1000);
            float end = -start;
            for (float i = start; i < end; i+=0.01f)
            {
                float x = i;
                float y = x*slope + intercept;
                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.scaledFrameWidth / 2) + (GlobVar.scaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.scaledFrameHeight / 2) + (GlobVar.scaledFrameHeight / 2));
                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
                if (index != -1)
                {
                    GlobVar.canvas[index] = 255;
                }
            }
        }
        public static void DrawPointNoPointcloud(float x, float y,float z)
        {
            var p = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };

            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(p);

            DrawPoint((int)depthSpacePoint.X/2,(int)depthSpacePoint.Y/2);
        }       
    }
}
