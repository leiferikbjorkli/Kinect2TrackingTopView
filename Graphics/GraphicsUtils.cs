//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Windows;
using System.Windows.Media;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class GraphicsUtils
    {
        public static void DrawRectangle(ThreePointRectangle rect )
        {
            int topLength = rect.b.x - rect.a.x;
            int sideLength = rect.c.y - rect.a.y;

            for (int i = 0; i < topLength; i++)
            {
                GlobVar.Canvas[GlobUtils.GetIndex(rect.a.x + i, rect.a.y)] = (byte)120;
                GlobVar.Canvas[GlobUtils.GetIndex(rect.c.x + i, rect.c.y)] = (byte)120;
            }
            for (int i = 0; i < sideLength; i++)
            {
                GlobVar.Canvas[GlobUtils.GetIndex(rect.a.x, rect.a.y + i)] = (byte)120;
                GlobVar.Canvas[GlobUtils.GetIndex(rect.b.x, rect.b.y + i)] = (byte)120;
            }
        }

        public static void DrawRectangle(IndexRectangle rect)
        {
            ThreePointRectangle pointRect = new ThreePointRectangle(GlobUtils.GetPoint(rect.a), GlobUtils.GetPoint(rect.b), GlobUtils.GetPoint(rect.c));
            DrawRectangle(pointRect);
        }

        public static void DrawPoint(CameraSpacePoint p)
        {
            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(p);

            DrawPoint((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2);
        }

        public static void DrawPoint(int x, int y)
        {
            int i;
            i = GlobUtils.GetIndex(x, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
        }

        public static void DrawPoint(int i)
        {
            DrawPoint(GlobUtils.GetPoint(i));
        }

        public static void DrawPoint(Point point)
        {
            int x = point.x;
            int y = point.y;

            int i;
            i = GlobUtils.GetIndex(x, y);
            if (GlobUtils.BoundaryCheck(i)){GlobVar.Canvas[i] = (byte)255;}
            i = GlobUtils.GetIndex(x + 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.Canvas[i] = (byte)255; }

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

        public static void DrawLine(float slope, float intercept,float r)
        {

            float start = -(float) (GlobVar.HalfMaxHorizontalWidth/1000);
            float end = -start;


            for (float i = start; i < end; i+=0.01f)
            {
                float x = i;
                float y = x*slope + intercept;

                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.HalfMaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.HalfMaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);

                if (index != -1)
                {
                    GlobVar.Canvas[index] = 255;
                }
            }
        }

        public static void DrawPoint(float x, float y)
        {
            // Can compute factor offline

            int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.HalfMaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.HalfMaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

            DrawPoint(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
        }

        public static void DrawLineNoPointcloud(float slope, float intercept,float r)
        {
            float start = -(float) (GlobVar.HalfMaxHorizontalWidth/1000);
            float end = -start;
            for (float i = start; i < end; i+=0.01f)
            {
                float x = i;
                float y = x*slope + intercept;
                int xPixelCoordinateOfPoint = (int)Math.Round(((x * 1000) / GlobVar.HalfMaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
                int yPixelCoordinateOfPoint = (int)Math.Round(((-y * 1000) / GlobVar.HalfMaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));
                int index = GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
                if (index != -1)
                {
                    GlobVar.Canvas[index] = 255;
                }
            }
        }

        public static void RenderDepthPixels(MainWindow mainWindow,CameraSpacePoint[] pointCloud)
        {

            byte[] intensityMap = KinectUtils.CalculateIntensityFromCameraSpacePoints(pointCloud);

            DrawCanvas(intensityMap);
            ClearCanvas();

            GlobVar.IntensityBitmap.WritePixels(
                new Int32Rect(0, 0, GlobVar.IntensityBitmap.PixelWidth, GlobVar.IntensityBitmap.PixelHeight),
                intensityMap,
                GlobVar.IntensityBitmap.PixelWidth * GlobVar.IntensityBitmap.Format.BitsPerPixel / 8,
                0);

            XAMLCanvas.DrawCanvas(mainWindow);
        }

        public static Color GetColorFromBodyId(int i)
        {
            switch (i)
            {
                case 1:
                    return Color.FromRgb(255, 0, 0);
                case 2:
                    return Color.FromRgb(0, 255, 0);
                case 3:
                    return Color.FromRgb(255, 255, 0);
                case 4:
                    return Color.FromRgb(0, 0, 255);
                case 5:
                    return Color.FromRgb(255, 130, 0);
            }
            return Color.FromRgb(255, 255, 255);
        }

        public static byte[] GetBgrColorFromBodyId(int bodyId)
        {
            switch (bodyId)
            {
                case 1:
                    return new byte[] { 0, 0, 255 };
                case 2:
                    return new byte[] { 0, 255, 0 };
                case 3:
                    return new byte[] { 0, 255, 255 };
                case 4:
                    return new byte[] { 255, 0, 0 };
                case 5:
                    return new byte[] { 0, 130, 255 };
            }
            return new byte[] { 0, 0, 0 };
        }
    }
}
