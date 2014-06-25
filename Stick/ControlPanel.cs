using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace StickDemo
{
    public class Slider : TrackBar
    {
        public String name;
        public Action<int> action;
        public Label label = new Label();
    }
    public class myButton : Button
    {
        public String name;
        public Action action;

    }
    public partial class ControlPanel : Form
    {
        public ControlPanel(int Height)
        {
            InitializeComponent();
            this.Width = 400;
            this.Height = Height;
        }

        private void ControlPanel_FormClosing(object sender, FormClosingEventArgs e)
        {
            e.Cancel = true;
        }
        public void addButton(string name, Action act, int x, int y)
        {
            myButton newButton = new myButton();
            newButton.name = name;
            newButton.action = act;
            this.Controls.Add(newButton);
            newButton.Top = y;
            newButton.Left = x;
            newButton.Height = 30;
            newButton.Width = 60;
            newButton.Text = newButton.name;
            newButton.Click += newButton_Click;
        }

        void newButton_Click(object sender, EventArgs e)
        {
            myButton button = (myButton)sender;
            if (button.action != null)
            {
                button.action();
            }
        }
        public void addSlider(string name, Action<int> act,int min,int max,int defVal,int x,int y)
        {
            Slider slider = new Slider();
            slider.action = act;
            slider.name = name;
            this.Controls.Add(slider);
            this.Controls.Add(slider.label);
            slider.label.Top = y;
            slider.label.Left = x + 200;
            slider.Top = y;
            slider.Left = x;
            slider.Width = 200;
            slider.Minimum = min;
            slider.Maximum = max;
            slider.Value = defVal;
            if (max - min < 50)
                slider.TickFrequency = 5;
            else if (max - min < 500)
                slider.TickFrequency = 50;
            else if (max - min < 5000)
                slider.TickFrequency = 500;
            else
                slider.TickFrequency = 5000;
            slider.ValueChanged += slider_ValueChanged;
            slider.action(slider.Value);
            slider.label.Text = slider.name + ": " + slider.Value.ToString();
            slider.label.Width = 200;
            
        }

        void slider_ValueChanged(object sender, EventArgs e)
        {
            Slider slider = (Slider)sender;
            if (slider.action != null)
            {
                slider.action(slider.Value);
                slider.label.Text = slider.name + ": "+slider.Value.ToString();
            }
        }
    }
}
