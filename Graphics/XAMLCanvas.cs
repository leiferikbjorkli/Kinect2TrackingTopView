//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows;
using System.Windows.Media.Imaging;

namespace InteractionDetection
{
    static class XAMLCanvas
    {
        private static Canvas canvas;

        public static void DrawCanvas(MainWindow window)
        {

            canvas = new Canvas();
            canvas.Width = 700;
            canvas.Height = 580;

            if (BodyUtils.HasBodyTracking())
            {
                //DrawBodies();
                //DrawEnergy();
            }

            canvas.Background = new ImageBrush(GlobVar.IntensityBitmap);
            window.Content = canvas;
            window.Show();
        }

        private static void DrawEnergy()
        {
            List<Body> lastBodies = GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count - 1);

            for (int i = 0; i < lastBodies.Count; i++)
            {

                var ellipseSize = lastBodies[i].EnergyLevel*100;
                int x = i*40+50;
                int y = 190;

                DrawEllipse(ellipseSize, x, y, GetColor(lastBodies[i].Id), 0.5);
                    
            }
        }

        private static void DrawBodies()
        {
            var ellipseSize = 20;

            foreach (var body in GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count-1))
            {
                Point pHead = GlobUtils.GetPoint(body.Head.CenterPointIndex);
                DrawEllipse(ellipseSize, pHead.x, pHead.y, GetColor(body.Id), 0.5);

                if (body.Hands[0] != null)
                {
                    Point p = KinectUtils.GetFramePointFromCameraSpace(body.Hands[0].CenterPoint);
                    DrawEllipse(ellipseSize, p.x, p.y, GetColor(4), 0.5);
                }
                if (body.Hands[1] != null)
                {
                    Point p = KinectUtils.GetFramePointFromCameraSpace(body.Hands[1].CenterPoint);
                    DrawEllipse(ellipseSize, p.x, p.y, GetColor(5), 0.5);
                }
            }


        }

        private static void DrawEllipse(double ellipseSize, double centerX, double centerY, Color color, double opacity)
        {
            Ellipse ellipse = new Ellipse { Width = ellipseSize, Height = ellipseSize };

            double left = centerX * (700.0 / (double)GlobVar.ScaledFrameWidth) - (ellipseSize / 2);
            double top = centerY * (580.0 / (double)GlobVar.ScaledFrameHeight) - (ellipseSize / 2);

            ellipse.Margin = new Thickness(left, top, 0, 0);

            SolidColorBrush solidColorBrush = new SolidColorBrush {Color = color};
            ellipse.Fill = solidColorBrush;
            ellipse.Opacity = opacity;

            canvas.Children.Add(ellipse);
        }

        private static Color GetColor(int i)
        {
            switch (i)
            {
                case 0:
                    return Color.FromRgb(255, 0, 255);
                case 1:
                    return Color.FromRgb(0, 255, 255);
                case 2:
                    return Color.FromRgb(255, 255, 255);
                case 3:
                    return Color.FromRgb(255, 255, 0);
                case 4:
                    return Color.FromRgb(255, 0, 0);
                case 5:
                    return Color.FromRgb(0, 0, 255);
                case 6:
                    return Color.FromRgb(0, 255, 0);
                case 7:
                    return Color.FromRgb(150, 100, 100);
                case 8:
                    return Color.FromRgb(255, 130, 0);
            }
            return Color.FromRgb(255, 255, 255);
        }

    }
}
