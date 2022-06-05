using System;
using System.Collections.Generic;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using System.Windows;

namespace Steiner
{
    internal class Graphic
    {
        private List<ScatterPoint> _points;
        public PlotModel Model { get; private set; }
        public Graphic()
        {
            var plotModel = new PlotModel
            {
                PlotAreaBorderThickness = new OxyThickness(1, 0, 0, 1),
                PlotMargins = new OxyThickness(0)
            };
            var series = new ScatterSeries
            {
                MarkerType = MarkerType.Circle,
                MarkerFill = OxyColors.Red
            };
            plotModel.Series.Add(series);
            var firstLinearAxis = new LinearAxis
            {
                IntervalLength = 20,
                TickStyle = TickStyle.Crossing,
                AxisDistance = -381.5,
                Minimum = -35,
                Maximum = 35,
                AxislineStyle = LineStyle.Solid
                //IsZoomEnabled = false
            };
            plotModel.Axes.Add(firstLinearAxis);
            var secondLinearAxis = new LinearAxis
            {
                Position = AxisPosition.Bottom,
                IntervalLength = 20,
                TickStyle = TickStyle.Crossing,
                AxislineStyle = LineStyle.Solid,
                AxisDistance = -253.5,
                Minimum = -50,
                Maximum = 50
                //IsZoomEnabled = false
            };
            plotModel.Axes.Add(secondLinearAxis);
            plotModel.PlotAreaBorderColor = OxyColors.Transparent;
            Model = plotModel;
            _points = series.Points;
        }
        public void AddPoint(double x, double y)
        {
            _points.Add(new ScatterPoint(x, y, 3));
            Model.InvalidatePlot(true);
        }
        public void AddPoints(IEnumerable<Point> points)
        {
            foreach(Point p in points)
            {
                _points.Add(new ScatterPoint(p.X, p.Y, 3));
            }
            Model.InvalidatePlot(true);
        }
        public void DeletePoint(int index)
        {
            _points.RemoveAt(index);
            Model.InvalidatePlot(true);
        }
        public void Clear()
        {
            _points.Clear();
            DeleteTree();
            Model.InvalidatePlot(true);
        }
        public void DrawSteinerTree(List<KeyValuePair<Point, List<Point>>> adjacencyList)
        {
            DeleteTree();
            List<KeyValuePair<Point, Point>> drawnEdges = new();
            foreach (var adjancency in adjacencyList)
            {
                foreach(var point in adjancency.Value)
                {
                    if (!drawnEdges.Exists(p => p.Key == adjancency.Key && p.Value == point || p.Key == point && p.Value == adjancency.Key))
                    {
                        FunctionSeries fs = new();
                        fs.Points.Add(new DataPoint(Math.Round(adjancency.Key.X, 12), Math.Round(adjancency.Key.Y, 12)));
                        fs.Points.Add(new DataPoint(Math.Round(point.X, 12), Math.Round(point.Y, 12)));
                        fs.Color = OxyColors.Blue;
                        Model.Series.Insert(0, fs);
                    }
                }
            }
            Model.InvalidatePlot(true);
        }
        private void DeleteTree()
        {
            for (int i = Model.Series.Count - 2; i >= 0; i--)
            {
                Model.Series.RemoveAt(i);
            }
        }
    }
}
