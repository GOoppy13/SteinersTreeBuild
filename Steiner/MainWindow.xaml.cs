using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Diagnostics;

namespace Steiner
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Graphic g = new();
        private List<Point> _points = new();
        private BackgroundWorker bw;
        private int _threadCount = 1;
        public MainWindow()
        {
            InitializeComponent();
            plotView.Model = g.Model;
            bw = new BackgroundWorker();
            bw.DoWork += bw_CreateSteinersTree;
        }
        private void bw_CreateSteinersTree(object sender, DoWorkEventArgs e)
        {
            string strThreadCount = Dispatcher.Invoke(() => textBoxThreadCount.Text);
            _threadCount = strThreadCount.Length == 0 ? _threadCount : Convert.ToInt32(strThreadCount);
            Stopwatch sw = new();
            sw.Start();
            g.DrawSteinerTree(AsyncSteinersTree.BuildSteinersTree(_points, _threadCount));
            sw.Stop();
            GC.Collect(GC.MaxGeneration);
            MessageBox.Show(sw.ElapsedMilliseconds.ToString());
        }
        private void pointsBtn_Click(object sender, RoutedEventArgs e)
        {
            (sender as Button).ContextMenu.IsEnabled = true;
            (sender as Button).ContextMenu.PlacementTarget = sender as Button;
            (sender as Button).ContextMenu.Placement = System.Windows.Controls.Primitives.PlacementMode.Bottom;
            (sender as Button).ContextMenu.IsOpen = true;
        }

        private void readFileBtn_Click(object sender, RoutedEventArgs e)
        {
            if (bw.IsBusy)
            {
                MessageBox.Show("Wait for the end of the calculation.");
                return;
            }
            OpenFileDialog ofd = new();
            if ((bool)!ofd.ShowDialog())
            {
                return;
            }
            List<Point> points = new();
            try
            {
                using (StreamReader sr = new(ofd.FileName))
                {
                    try
                    {
                        while (!sr.EndOfStream)
                        {
                            string[] values = sr.ReadLine().Split(new char[] { ' ', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
                            if (values.Length > 2)
                            {
                                throw new Exception();
                            }
                            Point point = new(Convert.ToDouble(values[0], new CultureInfo("en-US")), Convert.ToDouble(values[1], new CultureInfo("en-US")));
                            if (!_points.Contains(point))
                            {
                                points.Add(point);
                            }
                        }
                    }
                    catch(Exception)
                    {
                        MessageBox.Show("Incorrect data in the file.");
                        sr.Close();
                        return;
                    }
                }
            }
            catch(IOException)
            {
                MessageBox.Show("Cannot open chosen file.");
                return;
            }
            points = points.Distinct().ToList();
            _points.AddRange(points);
            g.AddPoints(points);
            points.ForEach(p => addInListPoint(p));
        }

        private void addPointBtn_Click(object sender, RoutedEventArgs e)
        {
            if (bw.IsBusy)
            {
                MessageBox.Show("Wait for the end of the calculation.");
                return;
            }
            AddPointForm apf = new();
            if ((bool)apf.ShowDialog())
            {
                Point p = new(apf.X, apf.Y);
                if (!_points.Contains(p))
                {
                    _points.Add(p);
                    g.AddPoint(p.X, p.Y);
                    addInListPoint(p);
                }
            }
        }
        private void calcBtn_Click(object sender, RoutedEventArgs e)
        {
            if (bw.IsBusy)
            {
                MessageBox.Show("Wait for the end of the calculation.");
                return;
            }
            if (_points.Count < 2)
            {
                MessageBox.Show("A minimum of 2 points is required to build.");
                return;
            }
            bw.RunWorkerAsync();
        }
        private void clearBtn_Click(object sender, RoutedEventArgs e)
        {
            if (bw.IsBusy)
            {
                MessageBox.Show("Wait for the end of the calculation.");
                return;
            }
            g.Clear();
            _points.Clear();
            pointsBtn.ContextMenu.Items.Clear();
        }
        private void addInListPoint(Point p)
        {
            MenuItem item = new()
            {
                Header = $"X:{p.X} Y:{p.Y}",
                Icon = (_points.IndexOf(p) + 1).ToString()
            };
            MenuItem deletePointBtn = new()
            {
                Header = "Delete"
            };
            deletePointBtn.Click += DeletePoint_Click;
            item.Items.Add(deletePointBtn);
            pointsBtn.ContextMenu.Items.Add(item);
        }
        private void DeletePoint_Click(object sender, RoutedEventArgs e)
        {
            if (bw.IsBusy)
            {
                MessageBox.Show("Wait for the end of the calculation.");
                return;
            }
            MenuItem m = (MenuItem)(sender as MenuItem).Parent;
            int index = Convert.ToInt32(m.Icon as string, new CultureInfo("en-US")) - 1;
            g.DeletePoint(index);
            _points.RemoveAt(index);
            pointsBtn.ContextMenu.Items.RemoveAt(index);
            RefreshIndex();
        }
        private void RefreshIndex()
        {
            for (int i = 0; i < pointsBtn.ContextMenu.Items.Count; i++)
            {
                (pointsBtn.ContextMenu.Items[i] as MenuItem).Icon = (i + 1).ToString();
            }
        }
        private void TextChangeThreadCount(object sender, TextCompositionEventArgs e)
        {
            if (!char.IsDigit(e.Text, 0) || bw.IsBusy)
            {
                e.Handled = true;
            }
        }

        private void PreviewKeyDownTextBoxHandler(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Space || bw.IsBusy)
            {
                e.Handled = true;
            }
        }
    }
}
