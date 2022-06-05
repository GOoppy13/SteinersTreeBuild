using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace Steiner
{
    /// <summary>
    /// Логика взаимодействия для AddPointForm.xaml
    /// </summary>
    public partial class AddPointForm : Window
    {
        public AddPointForm()
        {
            InitializeComponent();
        }
        private void Accept_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(x.Text, NumberStyles.Float, new CultureInfo("en-US"), out double xx) && double.TryParse(y.Text, NumberStyles.Float, new CultureInfo("en-US"), out double yy))
            {
                X = xx;
                Y = yy;
                DialogResult = true;
            }
        }
        public double X { get; private set; }
        public double Y { get; private set; }
        private void TextChange(object sender, TextCompositionEventArgs e)
        {
            if (!(char.IsDigit(e.Text, 0) 
               || (e.Text == ".")
               && !(sender as TextBox).Text.Contains(".")
               && (sender as TextBox).Text.Length != 0
               && (sender as TextBox).CaretIndex != 1
               || ((e.Text == ".") 
               && !(sender as TextBox).Text.Contains(".") 
               && (sender as TextBox).CaretIndex == 1 
               && (sender as TextBox).Text[0] != '-')
               || (e.Text == "-" && (sender as TextBox).CaretIndex == 0 
               && !(sender as TextBox).Text.Contains("-"))))
            {
                e.Handled = true;
            }
        }
    }
}
