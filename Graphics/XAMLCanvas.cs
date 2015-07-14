//
// Written by Leif Erik Bjoerkli
//


using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows;

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
                DrawEnergyValue();
            }

            canvas.Background = new ImageBrush(GlobVar.IntensityBitmap);
            window.Content = canvas;
            window.Show();
        }

        private static void DrawEnergyValue()
        {
            List<Body> lastBodies = GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count - 1);

            for (int i = 0; i < lastBodies.Count; i++)
            {

                TextBlock textBlock = new TextBlock();
                textBlock.Text = string.Format("{0:N2}", lastBodies[i].EnergyLevel);
                textBlock.Foreground = new SolidColorBrush(GraphicsUtils.GetColorFromBodyId(lastBodies[i].Id));
                textBlock.Margin = new Thickness(100, 200 + 20 * lastBodies[i].Id, 0, 0);
                canvas.Children.Add(textBlock);
            }
        }

        private static void DrawEnergy()
        {
            List<Body> lastBodies = GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count - 1);

            for (int i = 0; i < lastBodies.Count; i++)
            {

                var ellipseSize = lastBodies[i].EnergyLevel*100;
                int x = i*40+50;
                int y = 190;

                DrawEllipse(ellipseSize, x, y, GraphicsUtils.GetColorFromBodyId(lastBodies[i].Id), 0.5);
                    
            }
        }

        private static void DrawBodies()
        {
            var ellipseSize = 20;

            foreach (var body in GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count-1))
            {
                Point pHead = GlobUtils.GetPoint(body.Head.CenterPointIndex);
                DrawEllipse(ellipseSize, pHead.x, pHead.y, GraphicsUtils.GetColorFromBodyId(body.Id), 0.5);

                if (body.Hands[0] != null)
                {
                    Point p = KinectUtils.GetFramePointFromCameraSpace(body.Hands[0].CenterPoint);
                    DrawEllipse(ellipseSize, p.x, p.y, GraphicsUtils.GetColorFromBodyId(4), 0.5);
                }
                if (body.Hands[1] != null)
                {
                    Point p = KinectUtils.GetFramePointFromCameraSpace(body.Hands[1].CenterPoint);
                    DrawEllipse(ellipseSize, p.x, p.y, GraphicsUtils.GetColorFromBodyId(5), 0.5);
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

    }
}
