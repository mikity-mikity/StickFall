﻿using System;
using System.IO;
using System.Reflection;
using System.Windows.Forms;
using System.Xml;

namespace DemoFramework
{
    public class LibraryManager
    {
        static LibrarySelection librarySelection;

        const string SettingsFilename = "settings.xml";

        public static string GraphicsLibraryName
        {
            get;
            set;
        }

        static string[] supportedLibraries = new string[] { "SharpDX (DirectX 10)"/*, "OpenTK (OpenGL)"*/};
        public static string[] GetSupportedLibraries()
        {
            return supportedLibraries;
        }

        public static bool IsLibraryAvailable(string library)
        {
            string[] assemblyNames;
            switch (library)
            {
                case "SharpDX (DirectX 10)":
                    assemblyNames = new string[] { "DemoFramework.SharpDX", "SharpDX", "SharpDX.Direct3D10", "SharpDX.DXGI", "SharpDX.D3DCompiler" };
                    break;
                case "OpenTK (OpenGL)":
                    assemblyNames = new string[] { "DemoFramework.OpenTK", "OpenTK" };
                    break;
                default:
                    return false;
            }

            try
            {
                foreach (string assemblyName in assemblyNames)
                {
                    Assembly.Load(assemblyName);
                }
            }
            catch (Exception)
            {
                return false;
            }

            return true;
        }

        public static Graphics GetGraphics(Demo demo)
        {
            Type graphicsType;
            Assembly assembly;
            switch (LibraryManager.GraphicsLibraryName)
            {
                case "SharpDX (DirectX 10)":
                    assembly = Assembly.Load("DemoFramework.SharpDX");
                    graphicsType = assembly.GetType("DemoFramework.SharpDX.SharpDXGraphics");
                    break;
                case "OpenTK (OpenGL)":
                    assembly = Assembly.Load("DemoFramework.OpenTK");
                    graphicsType = assembly.GetType("DemoFramework.OpenTK.OpenTKGraphics");
                    break;
                default:
                    return null;
            }
            return graphicsType.InvokeMember(null, BindingFlags.CreateInstance, null, null, new[] { demo }) as Graphics;
        }

        static XmlElement GetSettingsDocumentRoot()
        {
            XmlDocument settings = new XmlDocument();
            XmlElement root;
            try
            {
                settings.Load(SettingsFilename);
                root = settings.DocumentElement;
            }
            catch (FileNotFoundException)
            {
                // Create an empty settings file
                root = settings.CreateElement("settings");
                settings.AppendChild(root);
                settings.Save(SettingsFilename);
            }

            return root;
        }

        public static void Initialize(Demo demo)
        {
            Application.EnableVisualStyles();

            // Check if BulletSharp exists
            try
            {
                Assembly.Load("BulletSharp");
            }
            catch (Exception e)
            {
                MessageBox.Show(e.ToString(), "BulletSharp Error!");
                return;
            }


            // Load settings
            XmlElement root = GetSettingsDocumentRoot();

            XmlNodeList l = root.GetElementsByTagName("graphicsframework");
            if (l.Count == 1)
            {
                XmlElement graphics = l.Item(0) as XmlElement;
                GraphicsLibraryName = graphics.GetAttribute("value");
            }

            if (GraphicsLibraryName == null)
            {
                if (!SelectLibrary())
                {
                    return;
                }
            }

            demo.Run();
            while (ExitWithReload)
            {
                ExitWithReload = false;
                if (!SelectLibrary())
                {
                    return;
                }
                demo.Run();
            }
        }

        public static bool SelectLibrary()
        {
            librarySelection = new LibrarySelection();
            librarySelection.ShowDialog();
            librarySelection.Dispose();
            librarySelection = null;
            return GraphicsLibraryName != null;
        }

        // Called by Graphics to indicate that the library was loaded successfully
        public static void LibraryStarted()
        {
            XmlElement root = GetSettingsDocumentRoot();

            XmlElement graphics;
            XmlNodeList l = root.GetElementsByTagName("graphicsframework");
            if (l.Count == 1)
            {
                graphics = l.Item(0) as XmlElement;
            }
            else
            {
                graphics = root.OwnerDocument.CreateElement("graphicsframework");
            }

            graphics.SetAttribute("value", GraphicsLibraryName);
            root.AppendChild(graphics);
            root.OwnerDocument.Save(SettingsFilename);
        }

        public static bool ExitWithReload
        {
            get;
            set;
        }
    }
}
