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

namespace InteractionDetection
{
    /// <summary>
    /// Interaction logic for StatisticsWindow.xaml
    /// </summary>
    public partial class StatisticsWindow : Window
    {
        private WriteableBitmap heatBitmap;
        private WriteableBitmap backgroundBitmap;


        public StatisticsWindow()
        {
            InitializeComponent();

            heatBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            backgroundBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);
        }


        public void Draw(byte[] heatMap, List<Dictionary<int, double>> energyHistory, List<TimeSpan> timestamps,List<int> handsTrackedHistory )
        {
            if (energyHistory.Count != timestamps.Count)
            {
                throw new Exception();
            }

            Grid grid = new Grid();
            grid.Margin = new Thickness(10, 10, 10, 10);

            RowDefinition rowDef1 = new RowDefinition();
            rowDef1.Height = new GridLength(520);

            RowDefinition rowDef2 = new RowDefinition();
            rowDef2.Height = new GridLength(200);

            grid.RowDefinitions.Add(rowDef1);
            grid.RowDefinitions.Add(rowDef2);

            DrawHeatMap(heatMap,ref grid);
            DrawEnergy(energyHistory,timestamps,ref grid, 200, SystemParameters.FullPrimaryScreenWidth, handsTrackedHistory);

            this.Content = grid;
            this.Show();
        }

        private double CalculateAverageEnergy(List<Dictionary<int, double>> energyHistory)
        {

            // Divide on total tracking frames

            int successFullyTrackedFrames = 0;
            double totalEnergy = 0;

            foreach (var frame in energyHistory)
            {
                if (frame.Count > 0)
                {
                    foreach (var energy in frame)
                    {
                        totalEnergy += energy.Value;
                    }
                    successFullyTrackedFrames++;
                }
                
            }
            return totalEnergy/successFullyTrackedFrames;
        }

        public void DrawHeatMap(byte[] heatMap, ref Grid grid)
        {
            backgroundBitmap.WritePixels(
                new Int32Rect(0, 0, backgroundBitmap.PixelWidth, backgroundBitmap.PixelHeight),
                KinectUtils.CalculateIntensityFromCameraSpacePoints(GlobVar.ScaledCameraSpacePoints),
                backgroundBitmap.PixelWidth * backgroundBitmap.Format.BitsPerPixel / 8,
                0);

            heatBitmap.WritePixels(
                new Int32Rect(0, 0, heatBitmap.PixelWidth, heatBitmap.PixelHeight),
                heatMap,
                heatBitmap.PixelWidth * heatBitmap.Format.BitsPerPixel / 8,
                0);

            Rectangle depthRect = new Rectangle();
            depthRect.Width = 603;
            depthRect.Height = 500;
            depthRect.Fill = new ImageBrush(backgroundBitmap);
            Grid.SetRow(depthRect,0);

            Rectangle heatRect = new Rectangle();
            heatRect.Width = 603;
            heatRect.Height = 500;
            heatRect.Fill = new ImageBrush(heatBitmap);
            Grid.SetRow(heatRect, 0);

            grid.Children.Add(depthRect);
            grid.Children.Add(heatRect);
            
        }

        public void DrawEnergy(List<Dictionary<int, double>> energyHistory, List<TimeSpan> timestamps, ref Grid grid, double rowHeight, double rowWidth, List<int> trackingHistory )
        {

            double graphCanvasWidth = rowWidth*0.9;
            double graphCanvasHeight = rowHeight*0.8;
            double timeCanvasWidth = graphCanvasWidth;
            double timeCanvasHeight = rowHeight*0.2;
            double energyAxisCanvasWidth = rowWidth*0.1;
            double energyAxisCanvasHeight = graphCanvasHeight;

            double averageEnergyUse = CalculateAverageEnergy(energyHistory);

            Grid subGrid = new Grid();
            Grid.SetRow(subGrid,1);

            RowDefinition row1 = new RowDefinition();
            row1.Height = new GridLength(graphCanvasHeight);
            RowDefinition row2 = new RowDefinition();
            row2.Height = new GridLength(timeCanvasHeight);
            subGrid.RowDefinitions.Add(row1);
            subGrid.RowDefinitions.Add(row2);

            ColumnDefinition col1 = new ColumnDefinition();
            col1.Width = new GridLength(energyAxisCanvasWidth);
            ColumnDefinition col2 = new ColumnDefinition();
            col2.Width = new GridLength(graphCanvasWidth);
            subGrid.ColumnDefinitions.Add(col1);
            subGrid.ColumnDefinitions.Add(col2);
            
            Canvas graphCanvas = new Canvas();
            Grid.SetRow(graphCanvas,0);
            Grid.SetColumn(graphCanvas,1);

            Canvas timestampCanvas = new Canvas();
            Grid.SetRow(timestampCanvas, 1);
            Grid.SetColumn(timestampCanvas,1);

            Canvas energyAxis = new Canvas();
            Grid.SetRow(energyAxis, 0);
            Grid.SetColumn(energyAxis, 0);

            grid.Children.Add(subGrid);
            subGrid.Children.Add(graphCanvas);
            subGrid.Children.Add(timestampCanvas);
            subGrid.Children.Add(energyAxis);

            double strokeThickness = rowWidth/energyHistory.Count;

            double maxEnergy = 0;

            foreach (var energyFrame in energyHistory)
            {
                foreach (var energy in energyFrame)
                {
                    double currentEnergy = energy.Value;
                    if (currentEnergy > maxEnergy)
                    {
                        maxEnergy = currentEnergy;
                    }

                }
            }
            maxEnergy = 50;

            DrawEnergyYAxis(ref energyAxis,maxEnergy,energyAxisCanvasHeight, energyAxisCanvasWidth,averageEnergyUse);
            DrawEnergyXAxis(ref timestampCanvas,trackingHistory,timeCanvasWidth);
            
            double energyScaling = (rowHeight*0.8)/maxEnergy;

            TimeSpan[] adjustedTimestamps = new TimeSpan[timestamps.Count];

            for (int i = 0; i < timestamps.Count; i++)
            {
                adjustedTimestamps[i] = timestamps[i].Subtract(timestamps[0]);
            }

            double distSinceLastTimestamp = rowWidth / 10;

            for (int i = 0; i < energyHistory.Count; i++)
            {
                if (distSinceLastTimestamp >= rowWidth/10)
                {
                    double x = strokeThickness*i;
                    DrawTimeStamp(ref timestampCanvas, adjustedTimestamps[i], x, timeCanvasHeight, timeCanvasWidth);
                    distSinceLastTimestamp = 0;
                }
                distSinceLastTimestamp += strokeThickness;

                double opacity = 1.0/energyHistory[i].Count;

                foreach (var energy in energyHistory[i])
	            {
		            double x1 = i*strokeThickness;
                    double y1 = 0;
                
                    double x2 = x1;
	                double y2 = energy.Value*energyScaling;

	                if (y2 > (maxEnergy*energyScaling))
	                {
	                    y2 = maxEnergy*energyScaling;
	                }

                    DrawEnergyBar(ref graphCanvas, x1, y1, x2, y2, graphCanvasHeight, strokeThickness,opacity,energy.Key); 
	            }
            }
        }

        private void DrawEnergyBar(ref Canvas canvas, double x1, double y1, double x2, double y2, double rowHeight, double strokeThickness,double opacity, int bodyId)
        {
            Line line = new Line();
            line.Stroke = new SolidColorBrush(GraphicsUtils.GetColorFromBodyId(bodyId));
            line.StrokeThickness = strokeThickness;
            line.Opacity = opacity;

            line.X1 = x1;
            line.X2 = x2;
            line.Y1 = rowHeight - y1;
            line.Y2 = rowHeight - y2;

            canvas.Children.Add(line);
        }

        private void DrawEnergyXAxis(ref Canvas canvas,List<int> trackingHistory, double canvasWidth)
        {

            double axisSegmentLength = canvasWidth/trackingHistory.Count;
            for (int i = 0; i < trackingHistory.Count; i++)
            {
                Line axis = new Line();
                axis.StrokeThickness = 3;
                axis.X1 = i *axisSegmentLength;
                axis.X2 = (i+1) * axisSegmentLength;
                axis.Y1 = 0;
                axis.Y2 = 0;

                if (trackingHistory[i] == 0)
                {
                    axis.Stroke = Brushes.Red;
                }
                else
                {
                    axis.Stroke = Brushes.Green;
                }
                canvas.Children.Add(axis);
            }
        }

        private void DrawTimeStamp(ref Canvas canvas, TimeSpan timestamp, double x, double canvasHeight, double canvasWidth)
        {
            TextBlock textBlock = new TextBlock();
            textBlock.Text = timestamp.ToString(@"mm\:ss");

            textBlock.Margin = new Thickness(x, 0, 0, 0);

            canvas.Children.Add(textBlock);
        }

        private void DrawEnergyYAxis(ref Canvas canvas, double maxEnergy, double canvasHeight, double canvasWidth, double totalEnergy)
        {
            TextBlock label = new TextBlock();
            label.Text = "Energy kg*m^2/s^2";
            label.Width = canvasWidth-30;
            label.TextWrapping = TextWrapping.Wrap;
            label.Margin = new Thickness(0, 0,0,0);
            canvas.Children.Add(label);

            TextBlock totalEnergyBlock = new TextBlock();
            totalEnergyBlock.Text = string.Format("Average energy used (KJoule): {0:N2}", totalEnergy);
            totalEnergyBlock.Width = canvasWidth - 40;
            totalEnergyBlock.TextWrapping = TextWrapping.Wrap;
            totalEnergyBlock.Margin = new Thickness(0, 50, 0, 0);
            canvas.Children.Add(totalEnergyBlock);

            Line axis = new Line();
            axis.StrokeThickness = 3;
            axis.Stroke = Brushes.Black;
            axis.X1 = canvasWidth;
            axis.X2 = canvasWidth;
            axis.Y1 = 0;
            axis.Y2 = canvasHeight;
            canvas.Children.Add(axis);

            int axisScaling = 5;

            for (int i = 0; i < axisScaling; i++)
            {
                double indexValue = (maxEnergy / axisScaling) * (axisScaling - i);
                TextBlock index = new TextBlock();
                index.Text = string.Format("{0:N2}", indexValue);
                index.Margin = new Thickness(canvasWidth - 30, i * (canvasHeight / axisScaling), 0, 0);
                canvas.Children.Add(index);
            }

        }

        // drawtracking success



        // The average value of torso and hand tracking are used to avoid the
        // energy calculations from being affected by noise in detection. 
        // Further, a three point median filter is used to filter out those
        // cases where the 
        private void ThreePointMedianSmoothingFilter()
        {
        }

        private void Stat_Window_Closing(object sender, CancelEventArgs e)
        {
            
        }
    }
}
