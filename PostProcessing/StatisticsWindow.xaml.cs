//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Interaction logic for StatisticsWindow.xaml. Shows stastics derived from sensing session.
    /// </summary>
    public partial class StatisticsWindow : Window
    {
        private readonly WriteableBitmap _heatBitmap;
        private readonly WriteableBitmap _backgroundBitmap;

        private const double RowHeightHeatMap = 520;
        private const double HeatMapHeight = 500;
        private const double HeatMapWidth = 603;

        const double MaxEnergy = 50;
        private const double RowHeightEnergyGraph = 200;
        private static double _rowWidthEnergyGraph;

        private static double _widthCanvasEnergyGraph;
        private static double _heightCanvasEnergyGraph;
        private static double _widthCanvasTimestampAxis;
        private static double _heightCanvasTimestampAxis;
        private static double _widthCanvasEnergyGraphYAxis;
        private static double _heightCanvasEnergyGraphYAxis;

        private static Grid _mainGrid;
        private static Grid _subGridUpper;
        private static Grid _subGridLower;

        private static Canvas _timestampCanvas;
        private static Canvas _energyYAxisCanvas;
        private static Canvas _energyGraphCanvas;
        private static Canvas _dataCanvas;
        
        public StatisticsWindow()
        {
            _heatBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            _backgroundBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);

            _rowWidthEnergyGraph = SystemParameters.FullPrimaryScreenWidth;

            _widthCanvasEnergyGraph = _rowWidthEnergyGraph * 0.9;
            _heightCanvasEnergyGraph = RowHeightEnergyGraph * 0.8;
            _widthCanvasTimestampAxis = _widthCanvasEnergyGraph;
            _heightCanvasTimestampAxis = RowHeightEnergyGraph * 0.2;
            _widthCanvasEnergyGraphYAxis = _rowWidthEnergyGraph * 0.1;
            _heightCanvasEnergyGraphYAxis = _heightCanvasEnergyGraph;

            SetupGrids();
            InitializeComponent();
        }

        /// <summary>
        /// Entry function for drawing the statistics window.
        /// </summary>
        public void Draw(byte[] heatMap, List<Dictionary<int, double>> energyHistory, List<TimeSpan> timestamps)
        {
            if (energyHistory.Count != timestamps.Count)
            {
                throw new Exception();
            }

            DrawHeatMap(heatMap);
            DrawEnergyGraph(energyHistory, timestamps);
            DrawAverageEnergyUse();
            DrawTrackingSuccess();

            Content = _mainGrid;
            Show();
        }

        /// <summary>
        /// Draws the heatmap on top of the last frame.
        /// </summary>
        private void DrawHeatMap(byte[] heatMap)
        {
            _backgroundBitmap.WritePixels(
                new Int32Rect(0, 0, _backgroundBitmap.PixelWidth, _backgroundBitmap.PixelHeight),
                ImageUtils.CalculateIntensityFromCameraSpacePoints(TemporalMedianImage.TemporalImage),
                _backgroundBitmap.PixelWidth * _backgroundBitmap.Format.BitsPerPixel / 8,
                0);

            _heatBitmap.WritePixels(
                new Int32Rect(0, 0, _heatBitmap.PixelWidth, _heatBitmap.PixelHeight),
                heatMap,
                _heatBitmap.PixelWidth * _heatBitmap.Format.BitsPerPixel / 8,
                0);

            var depthRect = new Rectangle
            {
                Width = HeatMapWidth,
                Height = HeatMapHeight,
                Fill = new ImageBrush(_backgroundBitmap)
            };
            Grid.SetRow(depthRect,0);
            Grid.SetColumn(depthRect,1);

            var heatRect = new Rectangle
            {
                Width = HeatMapWidth,
                Height = HeatMapHeight,
                Fill = new ImageBrush(_heatBitmap)
            };
            Grid.SetRow(heatRect, 0);
            Grid.SetColumn(heatRect, 1);

            _subGridUpper.Children.Add(depthRect);
            _subGridUpper.Children.Add(heatRect);
        }

        /// <summary>
        /// Draws the energy use with respect to time.
        /// </summary>
        private static void DrawEnergyGraph(List<Dictionary<int, double>> energyHistory, List<TimeSpan> timestamps)
        {
            DrawEnergyGraphYAxis();
            DrawEnergyGraphXAxis(energyHistory);
            DrawEnergyGraphXAxisTimeStamps(timestamps);
            DrawEnergyBars(energyHistory);
        }

        private static void DrawEnergyGraphXAxisTimeStamps(List<TimeSpan> timestamps)
        {
            var adjustedTimestamps = AdjustTimestamps(timestamps);
            var timestampXAxisScaling = _widthCanvasTimestampAxis / 10;
            var energyBarThickness = _widthCanvasTimestampAxis / timestamps.Count;

            for (var i = 0; i < timestamps.Count; i++)
            {
                if (timestampXAxisScaling >= _widthCanvasTimestampAxis / 10)
                {
                    var x = energyBarThickness * i;
                    DrawTimeStamp(adjustedTimestamps[i], x);
                    timestampXAxisScaling = 0;
                }
                timestampXAxisScaling += energyBarThickness;
            }
        }

        /// <remarks>
        /// If several bodies are tracked simultaneously, the opacity of each bar is adjusted so the colors of all bodies are visible.
        /// </remarks>>
        private static void DrawEnergyBars(List<Dictionary<int, double>> energyHistory)
        {
            var energyScaling = _heightCanvasEnergyGraphYAxis/MaxEnergy;
            var barThickness = _widthCanvasTimestampAxis/energyHistory.Count;

            for (var i = 0; i < energyHistory.Count; i++)
            {
                var opacity = 1.0/energyHistory[i].Count;

                foreach (var energy in energyHistory[i])
                {
                    var x1 = i*barThickness;
                    const double y1 = 0;

                    var x2 = x1;
                    var y2 = energy.Value*energyScaling;

                    if (y2 > (MaxEnergy*energyScaling))
                    {
                        y2 = MaxEnergy*energyScaling;
                    }

                    DrawBar(x1, y1, x2, y2, _heightCanvasEnergyGraph, barThickness, opacity, energy.Key);
                }
            }
        }

        /// <summary>
        /// Draws the average energy used in session.
        /// </summary>
        private static void DrawAverageEnergyUse()
        {
            var canvas = _dataCanvas;
            var averageEnergyUse = EnergyHistory.GetAverageEnergyUsed();
            var headlineBlock = new TextBlock
            {
                Text = "Average energy used (Joule/s):",
                Margin = new Thickness(30, 20, 0, 0),
                TextAlignment = TextAlignment.Center,
                TextWrapping = TextWrapping.Wrap,
                FontSize = 20
            };

            var averageEnergyBlock = new TextBlock
            {
                Text = string.Format("{0:N2}", averageEnergyUse),
                Foreground = Brushes.Red,
                Margin = new Thickness(30, 45, 0, 0),
                TextAlignment = TextAlignment.Center,
                FontSize = 20
            };
            canvas.Children.Add(headlineBlock);
            canvas.Children.Add(averageEnergyBlock);
        }

        private static void DrawTrackingSuccess()
        {
            var canvas = _dataCanvas;

            float headsTrackedAmount = TrackingDiagnostics.HeadsTrackedAmount*100;
            float torsosTrackedAmount = TrackingDiagnostics.TorsosTrackedAmount*100;

            var headsBlock = new TextBlock
            {
                Text = "HeadsTrackedSuccess:",
                Margin = new Thickness(30, 120, 0, 0),
                TextAlignment = TextAlignment.Center,
                TextWrapping = TextWrapping.Wrap,
                FontSize = 20
            };

            var headsTrackedBlock = new TextBlock
            {
                Text = string.Format("{0:N2}%", headsTrackedAmount),
                Foreground = Brushes.Red,
                Margin = new Thickness(30, 145, 0, 0),
                TextAlignment = TextAlignment.Center,
                FontSize = 20
            };

            var torsosBlock = new TextBlock
            {
                Text = "TorsosTrackedSuccess:",
                Margin = new Thickness(30, 220, 0, 0),
                TextAlignment = TextAlignment.Center,
                TextWrapping = TextWrapping.Wrap,
                FontSize = 20
            };

            var torsosTrackedBlock = new TextBlock
            {
                Text = string.Format("{0:N2}%", torsosTrackedAmount),
                Foreground = Brushes.Red,
                Margin = new Thickness(30, 245, 0, 0),
                TextAlignment = TextAlignment.Center,
                FontSize = 20
            };

            canvas.Children.Add(headsBlock);
            canvas.Children.Add(headsTrackedBlock);
            canvas.Children.Add(torsosBlock);
            canvas.Children.Add(torsosTrackedBlock);
        }

        private static void SetupGrids()
        {
            _mainGrid = new Grid { Margin = new Thickness(10, 10, 10, 10) };
            var rowHeatMap = new RowDefinition { Height = new GridLength(RowHeightHeatMap) };
            var rowEnergyHistory = new RowDefinition { Height = new GridLength(RowHeightEnergyGraph) };
            _mainGrid.RowDefinitions.Add(rowHeatMap);
            _mainGrid.RowDefinitions.Add(rowEnergyHistory);

            _subGridUpper = new Grid();
            Grid.SetRow(_subGridUpper, 0);
            _mainGrid.Children.Add(_subGridUpper);

            _subGridLower = new Grid();
            Grid.SetRow(_subGridLower, 1);
            _mainGrid.Children.Add(_subGridLower);

            var columnData = new ColumnDefinition() { Width = new GridLength(SystemParameters.FullPrimaryScreenWidth / 4) }; ;
            var columnHeatMap = new ColumnDefinition();
            _subGridUpper.ColumnDefinitions.Add(columnData);
            _subGridUpper.ColumnDefinitions.Add(columnHeatMap);

            _dataCanvas = new Canvas();
            Grid.SetRow(_dataCanvas, 0);
            Grid.SetColumn(_dataCanvas, 0);
            _subGridUpper.Children.Add(_dataCanvas);

            var rowEnergyGraphCanvas = new RowDefinition { Height = new GridLength(_heightCanvasEnergyGraph) };
            var rowTimestampCanvas = new RowDefinition {Height = new GridLength(_heightCanvasTimestampAxis)};
            _subGridLower.RowDefinitions.Add(rowEnergyGraphCanvas);
            _subGridLower.RowDefinitions.Add(rowTimestampCanvas);

            var colEnergyYAxis = new ColumnDefinition {Width = new GridLength(_widthCanvasEnergyGraphYAxis)};
            var colEnergyGraphCanvas = new ColumnDefinition {Width = new GridLength(_widthCanvasEnergyGraph)};
            _subGridLower.ColumnDefinitions.Add(colEnergyYAxis);
            _subGridLower.ColumnDefinitions.Add(colEnergyGraphCanvas);

            _energyGraphCanvas = new Canvas();
            Grid.SetRow(_energyGraphCanvas, 0);
            Grid.SetColumn(_energyGraphCanvas, 1);

            _timestampCanvas = new Canvas();
            Grid.SetRow(_timestampCanvas, 1);
            Grid.SetColumn(_timestampCanvas, 1);

            _energyYAxisCanvas = new Canvas();
            Grid.SetRow(_energyYAxisCanvas, 0);
            Grid.SetColumn(_energyYAxisCanvas, 0);
            
            _subGridLower.Children.Add(_energyGraphCanvas);
            _subGridLower.Children.Add(_timestampCanvas);
            _subGridLower.Children.Add(_energyYAxisCanvas);
        }

        /// <summary>
        /// Substracts the starting time from all timestamps so the time starts at 0.
        /// </summary>
        private static TimeSpan[] AdjustTimestamps(List<TimeSpan> timestamps)
        {
            var adjustedTimestamps = new TimeSpan[timestamps.Count];

            for (var i = 0; i < timestamps.Count; i++)
            {
                adjustedTimestamps[i] = timestamps[i].Subtract(timestamps[0]);
            }
            return adjustedTimestamps;
        }

        private static void DrawBar(double x1, double y1, double x2, double y2, double rowHeight, double barThickness,double opacity, int bodyId)
        {
            var canvas = _energyGraphCanvas;
            var line = new Line
            {
                Stroke = new SolidColorBrush(GraphicsUtils.GetColorFromBodyId(bodyId)),
                StrokeThickness = barThickness,
                Opacity = opacity,
                X1 = x1,
                X2 = x2,
                Y1 = rowHeight - y1,
                Y2 = rowHeight - y2
            };
            canvas.Children.Add(line);
        }

        /// <remarks>
        /// If a body was tracked at the given time, the axis is green. If no body was tracked, the axis is red.
        /// </remarks>>
        private static void DrawEnergyGraphXAxis(List<Dictionary<int, double>> energyHistory)
        {
            var axisSegmentLength = _widthCanvasEnergyGraph / energyHistory.Count;
            const int strokeThickness = 8;

            for (var i = 0; i < energyHistory.Count; i++)
            {
                var axis = new Line
                {
                    StrokeThickness = strokeThickness,
                    X1 = i*axisSegmentLength,
                    X2 = (i + 1)*axisSegmentLength,
                    Y1 = _heightCanvasEnergyGraph + strokeThickness / 2,
                    Y2 = _heightCanvasEnergyGraph + strokeThickness / 2,
                    Stroke = energyHistory[i].Count == 0 ? Brushes.Red : Brushes.Green
                };

                _energyGraphCanvas.Children.Add(axis);
            }
        }

        private static void DrawTimeStamp(TimeSpan timestamp, double x)
        {
            var textBlock = new TextBlock
            {
                Text = timestamp.ToString(@"mm\:ss"),
                Margin = new Thickness(x, 5, 0, 0)
            };
            _timestampCanvas.Children.Add(textBlock);
        }

        private static void DrawEnergyGraphYAxis()
        {
            var canvas = _energyYAxisCanvas;

            var label = new TextBlock
            {
                Text = "Energy J/frame",
                Width = _widthCanvasEnergyGraphYAxis - 30,
                TextWrapping = TextWrapping.Wrap,
                Margin = new Thickness(0, 0, 0, 0)
            };
            canvas.Children.Add(label);

            var strokeThickness = 4;
            var axis = new Line
            {
                StrokeThickness = strokeThickness,
                Stroke = Brushes.Black,
                X1 = _widthCanvasEnergyGraphYAxis + strokeThickness/2,
                X2 = _widthCanvasEnergyGraphYAxis + strokeThickness / 2,
                Y1 = 0,
                Y2 = _heightCanvasEnergyGraphYAxis
            };
            canvas.Children.Add(axis);

            const int axisScaling = 5;

            for (var i = 0; i < axisScaling; i++)
            {
                var indexValue = (MaxEnergy / axisScaling) * (axisScaling - i);
                var index = new TextBlock
                {
                    Text = string.Format("{0:N2}", indexValue),
                    Margin = new Thickness(_widthCanvasEnergyGraphYAxis - 30, i * (_heightCanvasEnergyGraphYAxis / axisScaling), 0, 0)
                };
                canvas.Children.Add(index);
            }
        }

        private void Stat_Window_Closing(object sender, CancelEventArgs e)
        {

        }
    }
}
