//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    static class GraphicsUtils
    {
        public static void DrawRectangle(IndexRectangle rect)
        {
            Point a = GlobUtils.GetPoint(rect.A);
            Point b = GlobUtils.GetPoint(rect.B);
            Point c = GlobUtils.GetPoint(rect.C);

            int topLength = b.X - a.X;
            int sideLength = c.Y - a.Y;

            for (int i = 0; i < topLength; i++)
            {
                GlobVar.GraphicsCanvas[GlobUtils.GetIndex(a.X + i, a.Y)] = (byte)120;
                GlobVar.GraphicsCanvas[GlobUtils.GetIndex(c.X + i, c.Y)] = (byte)120;
            }
            for (int i = 0; i < sideLength; i++)
            {
                GlobVar.GraphicsCanvas[GlobUtils.GetIndex(a.X, a.Y + i)] = (byte)120;
                GlobVar.GraphicsCanvas[GlobUtils.GetIndex(b.X, b.Y + i)] = (byte)120;
            }
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
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
        }

        public static void DrawPoint(int i)
        {
            DrawPoint(GlobUtils.GetPoint(i));
        }

        public static void DrawPoint(Point point)
        {
            int x = point.X;
            int y = point.Y;

            int i;
            i = GlobUtils.GetIndex(x, y);
            if (GlobUtils.BoundaryCheck(i)){GlobVar.GraphicsCanvas[i] = (byte)255;}
            i = GlobUtils.GetIndex(x + 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x + 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x - 1, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y + 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }
            i = GlobUtils.GetIndex(x, y - 1);
            if (GlobUtils.BoundaryCheck(i)) { GlobVar.GraphicsCanvas[i] = (byte)255; }

        }

        public static void DrawCanvasOnMap(byte[] intensityMap)
        {
            for (int i = 0; i < intensityMap.Length; i++)
            {
                if (GlobVar.GraphicsCanvas[i] != 0)
                {
                    intensityMap[i] = GlobVar.GraphicsCanvas[i];
                }
            }
        }

        public static void ClearCanvas()
        {
            for (int i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                GlobVar.GraphicsCanvas[i] = (byte)0;
            }
        }

        public static void RenderDepthPixels(MainWindow mainWindow,CameraSpacePoint[] pointCloud)
        {
            byte[] intensityMap = ImageUtils.CalculateIntensityFromCameraSpacePoints(pointCloud);

            DrawCanvasOnMap(intensityMap);
            ClearCanvas();

            GlobVar.IntensityBitmap.WritePixels(
                new Int32Rect(0, 0, GlobVar.IntensityBitmap.PixelWidth, GlobVar.IntensityBitmap.PixelHeight),
                intensityMap,
                GlobVar.IntensityBitmap.PixelWidth * GlobVar.IntensityBitmap.Format.BitsPerPixel / 8,
                0);

            var canvas = new Canvas
            {
                Width = 700,
                Height = 580
            };

            if (BodyUtils.HasBodyTracking() & Logger.DrawEnergyValue)
            {
                DrawEnergyValue(canvas);
            }

            canvas.Background = new ImageBrush(GlobVar.IntensityBitmap);
            mainWindow.Content = canvas;
            mainWindow.Show();
        }

        private static void DrawEnergyValue(Canvas canvas)
        {
            List<Body> lastBodies = BodiesHistory.Get.ElementAt(BodiesHistory.Get.Count - 1);

            foreach (var body in lastBodies)
            {
                var textBlock = new TextBlock
                {
                    Text = string.Format("{0:N2}", body.EnergyLevel),
                    Foreground = new SolidColorBrush(GetColorFromBodyId(body.Id)),
                    Margin = new Thickness(100, 200 + 20*body.Id, 0, 0)
                };
                canvas.Children.Add(textBlock);
            }
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
