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

            DrawBodies();

            canvas.Background = new ImageBrush(GlobVar.IntensityBitmap);
            window.Content = canvas;
            window.Show();
        }

        private static void DrawBodies()
        {
            foreach (var body in GlobVar.Bodies)
            {
                Point pHead = GlobUtils.GetPoint(body.Head.CenterPoint);
                DrawEllipse(30, 30, pHead.x, pHead.y, Color.FromArgb(255, 255, 255, 0),0.5);

                if (body.RightShoulder != null)
                {
                    Point pRightShoulder = GlobUtils.CalculateAveragePoint(body.RightShoulder.Points);
                    DrawEllipse(30, 30, pRightShoulder.x, pRightShoulder.y, Color.FromArgb(255, 100, 100, 0),0.5);
                }

                if (body.LeftShoulder != null)
                {
                    Point pLeftShoulder = GlobUtils.CalculateAveragePoint(body.LeftShoulder.Points);
                    DrawEllipse(30, 30, pLeftShoulder.x, pLeftShoulder.y, Color.FromArgb(255, 255, 100, 0),0.5);
                    
                }
                if (body.LeftHand != null)
                {
                    Point pLeftHand = GlobUtils.CalculateAveragePoint(body.LeftHand.Points);
                    DrawEllipse(30, 30, pLeftHand.x, pLeftHand.y, Color.FromArgb(255, 255, 0, 0), 1.0);

                }
                if (body.RightHand != null)
                {
                    Point pRightHand = GlobUtils.CalculateAveragePoint(body.RightHand.Points);
                    DrawEllipse(30, 30, pRightHand.x, pRightHand.y, Color.FromArgb(255, 100, 255, 0),1.0);
                }
            }


        }

        private static void DrawEllipse(double width, double height, double centerX, double centerY,Color color,double opacity)
        {
            Ellipse ellipse = new Ellipse { Width = width, Height = height };

            double left = centerX * (700.0 / (double)GlobVar.ScaledFrameWidth) - (width / 2);
            double top = centerY * (580.0 / (double)GlobVar.ScaledFrameHeight) - (height / 2);

            ellipse.Margin = new Thickness(left, top, 0, 0);

            SolidColorBrush solidColorBrush = new SolidColorBrush {Color = color};
            ellipse.Fill = solidColorBrush;
            ellipse.Opacity = opacity;

            canvas.Children.Add(ellipse);
        }

    }
}
