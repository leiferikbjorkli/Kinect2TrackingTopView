//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that draws graphics on the main window.
    /// </summary>
    static class XamlCanvas
    {
        private static Canvas _canvas;

        public static void Draw(MainWindow window)
        {
            _canvas = new Canvas();
            _canvas.Width = 700;
            _canvas.Height = 580;

            if (BodyUtils.HasBodyTracking())
            {
                DrawEnergyValue();
            }

            _canvas.Background = new ImageBrush(GlobVar.IntensityBitmap);
            window.Content = _canvas;
            window.Show();
        }

        private static void DrawEnergyValue()
        {
            List<Body> lastBodies = BodiesHistory.Get.ElementAt(BodiesHistory.Get.Count - 1);

            for (int i = 0; i < lastBodies.Count; i++)
            {
                TextBlock textBlock = new TextBlock();
                textBlock.Text = string.Format("{0:N2}", lastBodies[i].EnergyLevel);
                textBlock.Foreground = new SolidColorBrush(GraphicsUtils.GetColorFromBodyId(lastBodies[i].Id));
                textBlock.Margin = new Thickness(100, 200 + 20 * lastBodies[i].Id, 0, 0);
                _canvas.Children.Add(textBlock);
            }
        }
    }
}
