using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
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
                GlobVar.Canvas[GlobUtils.GetIndex(xStart + i, yStart)] = (byte)255;
                GlobVar.Canvas[GlobUtils.GetIndex(xStart + i, yStart + rect.Height)] = (byte)255;
            }
            for (int j = 0; j < rect.Height; j++)
            {
                GlobVar.Canvas[GlobUtils.GetIndex(xStart, yStart + j)] = (byte)255;
                GlobVar.Canvas[GlobUtils.GetIndex(xStart + rect.Width, yStart + j)] = (byte)255;
            }
        }

        public static void DrawRectangle(ThreePointRectangle rect )
        {
            int topLength = rect.b.x - rect.a.x;
            int sideLength = rect.c.y - rect.a.y;

            for (int i = 0; i < topLength; i++)
            {
                GlobVar.Canvas[GlobUtils.GetIndex(rect.a.x + i, rect.a.y)] = (byte)255;
                GlobVar.Canvas[GlobUtils.GetIndex(rect.c.x + i, rect.c.y)] = (byte)255;
            }
            for (int i = 0; i < sideLength; i++)
            {
                GlobVar.Canvas[GlobUtils.GetIndex(rect.a.x, rect.a.y + i)] = (byte)255;
                GlobVar.Canvas[GlobUtils.GetIndex(rect.b.x, rect.b.y + i)] = (byte)255;
            }
        }
        public static void DrawRectangle(IndexRectangle rect)
        {
            ThreePointRectangle pointRect = new ThreePointRectangle(GlobUtils.GetPoint(rect.a), GlobUtils.GetPoint(rect.b), GlobUtils.GetPoint(rect.c));
            DrawRectangle(pointRect);
        }


        public static void DrawPoint(int x, int y)
        {
            GlobVar.Canvas[GlobUtils.GetIndex(x, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y - 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y - 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x, y - 1)] = (byte)255;
        }
        public static void DrawPoint(Point point)
        {
            int x = point.x;
            int y = point.y;

            GlobVar.Canvas[GlobUtils.GetIndex(x, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y - 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x + 1, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y - 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x - 1, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x, y + 1)] = (byte)255;
            GlobVar.Canvas[GlobUtils.GetIndex(x, y - 1)] = (byte)255;
        }

        public static void DrawCanvas(byte[] intensityMap)
        {
            for (int i = 0; i < intensityMap.Length; i++)
            {
                if (GlobVar.Canvas[i] != 0)
                {
                    intensityMap[i] = GlobVar.Canvas[i];
                }
            }
        }

        public static void ClearCanvas()
        {
            for (int i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                GlobVar.Canvas[i] = (byte)0;
            }
        }

        public static void ConvertFromGrayscaleToRgb()
        {
        }


        public static void DrawBodies()
        {

            foreach (var person in GlobVar.Bodies)
            {
                if (person.LeftHand.Points != null)
                {
                    foreach (var p in person.LeftHand.Points)
                    {

                        if (GlobVar.Canvas[p] == 0)
                        {
                            GlobVar.Canvas[p] = (byte) 255;
                        }
                    }
                    foreach (var p in person.RightHand.Points)
                    {
                        if (GlobVar.Canvas[p] == 0)
                        {
                            GlobVar.Canvas[p] = (byte)150;
                        }
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

                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);

                if (index != -1)
                {
                    GlobVar.Canvas[index] = 255;
                }
            }
        }

        public static void DrawPoint(int i)
        {
            GlobVar.Canvas[i] = 255;
        }

        public static void DrawPoint(float x, float y)
        {

            // Can compute factor offline

            int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

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
                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));
                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
                if (index != -1)
                {
                    GlobVar.Canvas[index] = 255;
                }
            }
        }

        public static void RenderDepthPixels(MainWindow mainWindow)
        {

            byte[] intensityMap = KinectUtils.CalculateIntensityFromCameraSpacePoints(GlobVar.MedianFilteredPointCloud);

            DrawCanvas(intensityMap);
            ClearCanvas();

            GlobVar.IntensityBitmap.WritePixels(
                new Int32Rect(0, 0, GlobVar.IntensityBitmap.PixelWidth, GlobVar.IntensityBitmap.PixelHeight),
                intensityMap,
                GlobVar.IntensityBitmap.PixelWidth * GlobVar.IntensityBitmap.Format.BitsPerPixel / 8,
                0);

            XAMLCanvas.DrawCanvas(mainWindow);
        }
    }
}
