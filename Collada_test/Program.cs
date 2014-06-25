using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using grendgine_collada;
using System.IO;
using System.Xml;
using System.Xml.Serialization;

namespace Collada_test
{
    class Program
    {
        static void export()
        {
            Grendgine_Collada col_scenes = null;
            col_scenes = new Grendgine_Collada();
            //col_scenes = (Grendgine_Collada)sr.Deserialize(tr2);
            /*string file_name2 = "c:/out/test2.dae";
            TextReader tr2 = new StreamReader(file_name2);
            tr2.Close();
            */
            col_scenes.Collada_Version = "1.4.1";
            col_scenes.Asset = new Grendgine_Collada_Asset();
            col_scenes.Asset.Contributor = new Grendgine_Collada_Asset_Contributor[1];
            col_scenes.Asset.Contributor[0] = new Grendgine_Collada_Asset_Contributor();
            col_scenes.Asset.Contributor[0].Author = "G30";
            col_scenes.Asset.Contributor[0].Authoring_Tool = "something";
            col_scenes.Asset.Created = System.DateTime.Now;
            col_scenes.Asset.Modified = System.DateTime.Now;
            col_scenes.Asset.Unit = new Grendgine_Collada_Asset_Unit();
            col_scenes.Asset.Unit.Meter = 0.025400;
            col_scenes.Asset.Unit.Name = "centimeter";
            col_scenes.Asset.Up_Axis = "Y_UP";
            col_scenes.Library_Geometries = new Grendgine_Collada_Library_Geometries();
            col_scenes.Library_Geometries.Geometry=new Grendgine_Collada_Geometry[1];
            col_scenes.Library_Geometries.Geometry[0] = new Grendgine_Collada_Geometry();
            Grendgine_Collada_Geometry geom = col_scenes.Library_Geometries.Geometry[0];
            string uniqueID="BOX01";
            geom.ID = uniqueID+"-lib";
            geom.Name = uniqueID+"Mesh";
            geom.Mesh = new Grendgine_Collada_Mesh();
            geom.Mesh.Source = new Grendgine_Collada_Source[1];
            for (int i = 0; i < 1; i++)
            {
                geom.Mesh.Source[i]=new Grendgine_Collada_Source();
            }
            geom.Mesh.Source[0].ID = uniqueID + "-POSITION";
            geom.Mesh.Source[0].Float_Array = new Grendgine_Collada_Float_Array();
            geom.Mesh.Source[0].Float_Array.Count = 24;
            geom.Mesh.Source[0].Float_Array.ID = uniqueID + "-POSITION-array";
            //Positions of Vertices
            geom.Mesh.Source[0].Float_Array.Value_As_String = "0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0 1.0 1.0 0.0 0.0 0.0 1.0 1.0 0.0 1.0 0.0 1.0 1.0 1.0 1.0 1.0";
            geom.Mesh.Source[0].Technique_Common = new Grendgine_Collada_Technique_Common_Source();
            geom.Mesh.Source[0].Technique_Common.Accessor = new Grendgine_Collada_Accessor();
            geom.Mesh.Source[0].Technique_Common.Accessor.Source = "#" + geom.Mesh.Source[0].Float_Array.ID;
            geom.Mesh.Source[0].Technique_Common.Accessor.Stride = 3;
            geom.Mesh.Source[0].Technique_Common.Accessor.Count = (uint)geom.Mesh.Source[0].Float_Array.Count / geom.Mesh.Source[0].Technique_Common.Accessor.Stride;
            geom.Mesh.Source[0].Technique_Common.Accessor.Param = new Grendgine_Collada_Param[3];
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[0] = new Grendgine_Collada_Param();
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[0].Name = "X";
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[0].Type = "float";
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[1] = new Grendgine_Collada_Param();
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[1].Name = "Y";
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[1].Type = "float";
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[2] = new Grendgine_Collada_Param();
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[2].Name = "Z";
            geom.Mesh.Source[0].Technique_Common.Accessor.Param[2].Type = "float";

            geom.Mesh.Vertices = new Grendgine_Collada_Vertices();
            geom.Mesh.Vertices.ID = uniqueID + "-VERTEX";
            geom.Mesh.Vertices.Input = new Grendgine_Collada_Input_Unshared[1];
            geom.Mesh.Vertices.Input[0] = new Grendgine_Collada_Input_Unshared();
            geom.Mesh.Vertices.Input[0].Semantic = Grendgine_Collada_Input_Semantic.POSITION;
            geom.Mesh.Vertices.Input[0].source = "#" + geom.Mesh.Source[0].ID;
            geom.Mesh.Triangles = new Grendgine_Collada_Triangles[1];
            geom.Mesh.Triangles[0] = new Grendgine_Collada_Triangles();
            geom.Mesh.Triangles[0].Count = 12;
            geom.Mesh.Triangles[0].Input = new Grendgine_Collada_Input_Shared[1];
            geom.Mesh.Triangles[0].Input[0] = new Grendgine_Collada_Input_Shared();
            geom.Mesh.Triangles[0].Input[0].Semantic = Grendgine_Collada_Input_Semantic.VERTEX;
            geom.Mesh.Triangles[0].Input[0].Offset = 0;
            geom.Mesh.Triangles[0].Input[0].source = "#" + geom.Mesh.Vertices.ID;
            geom.Mesh.Triangles[0].P = new Grendgine_Collada_Int_Array_String();
            geom.Mesh.Triangles[0].P.Value_As_String = "0 1 2 2 1 3 1 5 3 3 5 7 4 0 6 6 0 2 5 4 7 7 4 6 0 4 1 1 4 5 2 3 6 6 3 7";

            col_scenes.Library_Visual_Scene = new Grendgine_Collada_Library_Visual_Scenes();
            col_scenes.Library_Visual_Scene.Visual_Scene = new Grendgine_Collada_Visual_Scene[1];
            col_scenes.Library_Visual_Scene.Visual_Scene[0] = new Grendgine_Collada_Visual_Scene();
            Grendgine_Collada_Visual_Scene scene=col_scenes.Library_Visual_Scene.Visual_Scene[0];
            scene.ID = "test";
            scene.Name = "test";
            scene.Node = new Grendgine_Collada_Node[1];
            scene.Node[0] = new Grendgine_Collada_Node();
            scene.Node[0].Name = uniqueID;
            scene.Node[0].ID = uniqueID;
            scene.Node[0].sID = uniqueID;
            scene.Node[0].Type = Grendgine_Collada_Node_Type.NODE;
            scene.Node[0].Instance_Geometry = new Grendgine_Collada_Instance_Geometry[1];
            scene.Node[0].Instance_Geometry[0] = new Grendgine_Collada_Instance_Geometry();
            scene.Node[0].Instance_Geometry[0].URL = "#" + geom.ID;
            col_scenes.Scene = new Grendgine_Collada_Scene();
            col_scenes.Scene.Visual_Scene = new Grendgine_Collada_Instance_Visual_Scene();
            col_scenes.Scene.Visual_Scene.URL = "#" + scene.ID;
            XmlSerializer sr = new XmlSerializer(typeof(Grendgine_Collada));
            string file_name = "c:/out/test.dae";
            TextWriter tr = new StreamWriter(file_name);
            sr.Serialize(tr, col_scenes);

            tr.Close();
        }
        static void Main(string[] args)
        {
            Program.export();
        }
    }
}
