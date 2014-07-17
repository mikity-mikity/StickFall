using System;
using BulletSharp;
using BulletSharp.MultiThreaded;
using DemoFramework;
using System.Windows.Forms;
using System.Collections.Generic;
using grendgine_collada;
using System.IO;
using System.Xml;
using System.Xml.Serialization;
namespace StickDemo
{
    class StickDemo : Demo
    {
        double Interval = 1.0d;
        float scale = 0.1f;
        int count = 0;
        int pathPtCount = 0;
        BoxShape box = null;
        RigidBody stickSample;
        //Vector3 pathStPt, pathEndPtU, pathEndPtV;
        List<Vector3> pos;
        List<double[]> points;
        Vector3[] _pos;
        List<RigidBody> manyBoxes;
        //Variables
        float gravity;
        float friction;
        //stick
        float stickMass;
        float stickSizeX;
        float stickSizeY;
        float stickSizeZ;
        float releaseHt;
        //base
        float baseW;
        float baseD;
        float baseH;
        float baseWallThickness;
        int obstacleX;
        int obstacleY;
        float obstacleW;
        float obstacleD;
        float obstacleH;
        //Path
        //float pathStX;
        //float pathEndX;
        //int pathDiv = 100;
        List<RigidBody> sticks = new List<RigidBody>();
        List<RigidBody> frozenSticks = new List<RigidBody>();
        List<RigidBody> baseBoxes = new List<RigidBody>();
        void import()
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "TXT(*.txt)|*.txt|Any type(*.*)|*.*";
            ofd.FilterIndex = 0;
            ofd.Title = "File Open";
            ofd.RestoreDirectory = true;
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                string file_name = ofd.FileName;
                TextReader tr = new StreamReader(file_name);
                List<double> elems = new List<double>();
                List<double> sortedList = new List<double>();
                string line;
                points = new List<double[]>();
                while ((line = tr.ReadLine()) != null)
                {
                    string[] vals=line.Split(',');
                    if (vals.Length == 3)
                    {
                        points.Add(new double[3] { Double.Parse(vals[0]) * 0.1d, Double.Parse(vals[1]) * 0.1d, Double.Parse(vals[2]) * 0.1d });
                    }
                }
                tr.Close();
            }

        }
        void export()
        {
            double global_scale = 10d;
            Grendgine_Collada col_scenes = null;
            col_scenes = new Grendgine_Collada();
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
            List<RigidBody> boxesAll = new List<RigidBody>();
            boxesAll.AddRange(sticks);
            boxesAll.AddRange(frozenSticks);
            boxesAll.AddRange(baseBoxes);
            col_scenes.Library_Geometries.Geometry = new Grendgine_Collada_Geometry[boxesAll.Count];
            for (int i = 0; i < boxesAll.Count; i++)
            {
                col_scenes.Library_Geometries.Geometry[i] = new Grendgine_Collada_Geometry();
                Grendgine_Collada_Geometry geom = col_scenes.Library_Geometries.Geometry[i];
                string uniqueID = "Box"+i.ToString();
                geom.ID = uniqueID + "-lib";
                geom.Name = uniqueID + "Mesh";
                geom.Mesh = new Grendgine_Collada_Mesh();
                geom.Mesh.Source = new Grendgine_Collada_Source[1];
                for (int j = 0; j < 1; j++)
                {
                    geom.Mesh.Source[j] = new Grendgine_Collada_Source();
                }
                geom.Mesh.Source[0].ID = uniqueID + "-POSITION";
                geom.Mesh.Source[0].Float_Array = new Grendgine_Collada_Float_Array();
                geom.Mesh.Source[0].Float_Array.ID = uniqueID + "-POSITION-array";
                //Positions of Vertices
                //ConvexShape
                int nV=0;
                int nF=0;
                if (boxesAll[i].CollisionShape.IsPolyhedral && boxesAll[i].CollisionShape.IsConvex)
                {
                    if (boxesAll[i].CollisionShape is BoxShape)
                    {
                        BoxShape shape = (BoxShape)boxesAll[i].CollisionShape;
                        Vector3 size = shape.HalfExtentsWithMargin;
                        double[][] vertices = new double[8][];
                        for (int x = -1; x < 2; x+=2)
                        {
                            for (int y = -1; y < 2; y += 2)
                            {
                                for (int z = -1; z < 2; z += 2)
                                {
                                    int index = ((x + 1) / 2) * 4 + ((y + 1) / 2) * 2 + ((z + 1) / 2) * 1;
                                    vertices[index] = new double[3] { size.X * x * global_scale, size.Y * y * global_scale, size.Z * z * global_scale };
                                }
                            }
                        }
                        nV = 8;
                        nF = 12;
                        geom.Mesh.Source[0].Float_Array.Count = nV * 3;
                        string val = "";
                        for (int j = 0; j < 8; j++)
                        {
                            for (int k = 0; k < 3; k++)
                            {
                                val += "  " + vertices[j][k].ToString();
                            }
                        }
                        geom.Mesh.Source[0].Float_Array.Value_As_String = val;
                    }
                }
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
                geom.Mesh.Triangles[0].Count = nF*2;
                geom.Mesh.Triangles[0].Input = new Grendgine_Collada_Input_Shared[1];
                geom.Mesh.Triangles[0].Input[0] = new Grendgine_Collada_Input_Shared();
                geom.Mesh.Triangles[0].Input[0].Semantic = Grendgine_Collada_Input_Semantic.VERTEX;
                geom.Mesh.Triangles[0].Input[0].Offset = 0;
                geom.Mesh.Triangles[0].Input[0].source = "#" + geom.Mesh.Vertices.ID;
                geom.Mesh.Triangles[0].P = new Grendgine_Collada_Int_Array_String();
                geom.Mesh.Triangles[0].P.Value_As_String = "0 1 2 2 1 3 1 5 3 3 5 7 4 0 6 6 0 2 5 4 7 7 4 6 0 4 1 1 4 5 2 3 6 6 3 7";
            }

            col_scenes.Library_Visual_Scene = new Grendgine_Collada_Library_Visual_Scenes();
            col_scenes.Library_Visual_Scene.Visual_Scene = new Grendgine_Collada_Visual_Scene[1];
            col_scenes.Library_Visual_Scene.Visual_Scene[0] = new Grendgine_Collada_Visual_Scene();
            Grendgine_Collada_Visual_Scene scene = col_scenes.Library_Visual_Scene.Visual_Scene[0];
            scene.ID = "test";
            scene.Name = "test";

            scene.Node = new Grendgine_Collada_Node[boxesAll.Count];
            for (int i = 0; i < boxesAll.Count; i++)
            {
                string uniqueID = "Box" + i.ToString();
                Grendgine_Collada_Geometry geom = col_scenes.Library_Geometries.Geometry[i];
                Matrix T = boxesAll[i].WorldTransform;

                scene.Node[i] = new Grendgine_Collada_Node();
                scene.Node[i].Name = uniqueID;
                scene.Node[i].ID = uniqueID;
                scene.Node[i].sID = uniqueID;
                scene.Node[i].Type = Grendgine_Collada_Node_Type.NODE;
                scene.Node[i].Instance_Geometry = new Grendgine_Collada_Instance_Geometry[1];
                scene.Node[i].Instance_Geometry[0] = new Grendgine_Collada_Instance_Geometry();
                scene.Node[i].Instance_Geometry[0].URL = "#" + geom.ID;
                scene.Node[i].Matrix = new Grendgine_Collada_Matrix[1];
                scene.Node[i].Matrix[0] = new Grendgine_Collada_Matrix();
                scene.Node[i].Matrix[0].sID = "matrix";
                string val = "";
                for (int j = 0; j < 4; j++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        if (k == 3 && (j == 0 || j == 1 || j == 2))
                        {
                            val += "  " + (T[k, j]*global_scale).ToString();
                        }
                        else
                        {
                            val += "  " + T[k, j].ToString();
                        }
                    }
                }
                scene.Node[i].Matrix[0].Value_As_String = val;

                col_scenes.Scene = new Grendgine_Collada_Scene();
                col_scenes.Scene.Visual_Scene = new Grendgine_Collada_Instance_Visual_Scene();
                col_scenes.Scene.Visual_Scene.URL = "#" + scene.ID;
            }
            XmlSerializer sr = new XmlSerializer(typeof(Grendgine_Collada));
            SaveFileDialog sfd = new SaveFileDialog();
            sfd.FileName = "sticks.dae";
            sfd.Filter = "COLLADA(*.dae)|*.dae|Any type(*.*)|*.*";
            sfd.FilterIndex = 0;
            sfd.Title = "File Open";
            sfd.RestoreDirectory = true;
            if (sfd.ShowDialog() == DialogResult.OK)
            {
                string file_name = sfd.FileName;
                TextWriter tr = new StreamWriter(file_name);
                sr.Serialize(tr, col_scenes);
                tr.Close();
            }
            
        }
        void dropStickAlongPath(Vector3 pathPt)
        {
            Vector3 pos = pathPt;
            int N = 10;
            for(int i=0;i<N+1;i++)
            {
                Vector3 dPos = pos + new Vector3((i - N/2f)/N*2.4f, 0, 0);
                RigidBody cmbody = LocalCreateRigidBody(stickMass, Matrix.Translation(dPos), box);
                cmbody.Friction = friction;
                count++;
                sticks.Add(cmbody);
            }

        }
        public Vector3 tmpTarget;
        //Similar to draw()
        public override void OnUpdate()
        {
            for (int i = 0; i < sticks.Count; i++)
            {
                RigidBody stick = sticks[i];
                Vector3 v = stick.LinearVelocity;
                if (v.LengthSquared() < 0.001)
                {
                    stick.SetMassProps(0, new Vector3(0, 0, 0));
                    stick.UserObject = "static";
                    frozenSticks.Add(stick);
                    sticks.Remove(stick);                    
                }
            }
            if (realTime>Interval)
            {
                realTime -= Interval;
                if (run) //When R is pressed, run turns to true. When it is pressed again, run turns to false and dropping sticks stops, but simulations goes.
                {
                    dropStickAlongPath(pos[pathPtCount]);
                    this.tmpTarget = pos[pathPtCount];
                    pathPtCount++;
                    if (pathPtCount >= pos.Count)
                    {
                        pathPtCount = 0;
                    }
                }
            }
            Freelook.SetEyeTarget(Freelook.Eye, new Vector3(Freelook.Target.X * 0.8f + tmpTarget.X * 0.2f, Freelook.Target.Y * 0.8f + tmpTarget.Y * 0.2f, Freelook.Target.Z * 0.8f + tmpTarget.Z * 0.2f));
            Graphics.SetInfoText("Press R to start simulation\n" +
                                    "F3 - Toggle debug\n" +
                                    "Space - Shoot box\n"+
                                    "Total boxes:"+count);
            base.OnUpdate();
        }

        //Camera settings
        Vector3 eye = new Vector3(0,150 , -100);
        Vector3 target = new Vector3(0, 30, 0);
        bool UseParallelDispatcherBenchmark = false;   //Best setting in performance

        float defaultContactProcessingThreshold = 0.0f;
        ControlPanel cp5;

        //Similar to setup()
        protected override void OnInitialize()
        {
            cp5 = new ControlPanel(900);
            cp5.addSlider("baseW", (val) => { baseW = (float)val * scale; renewBase(); }, 1, 2000, 1000, 20, 40);
            cp5.addSlider("baseD", (val) => { baseD = (float)val * scale*0.3f; renewBase(); }, 1, 500, 70, 20, 90);
            cp5.addSlider("baseH", (val) => { baseH = (float)val * scale*0.4f; renewBase(); }, 1, 500, 50, 20, 140);
            cp5.addSlider("baseWallThickness", (val) => { baseWallThickness = (float)val * scale*0.4f; renewBase(); }, 1, 30, 10, 20, 190);

            cp5.addSlider("obstacleX", (val) => { obstacleX = val; renewBase(); }, 1, 20, 8, 20, 240);
            cp5.addSlider("obstacleY", (val) => { obstacleY = val; renewBase(); }, 1, 10, 1, 20, 290);
            cp5.addSlider("obstacleW", (val) => { obstacleW = (float)val * scale; renewBase(); }, 1, 50, 10, 20, 340);
            cp5.addSlider("obstacleD", (val) => { obstacleD = (float)val * scale; renewBase(); }, 1, 50, 10, 20, 390);
            cp5.addSlider("obstacleH", (val) => { obstacleH = (float)val * scale; renewBase(); }, 0, 1000, 200, 20, 440);

            cp5.addSlider("stickSizeX", (val) => { stickSizeX = (float)val * scale*0.1f; renewStick(); }, 1, 50, 5, 20, 490);
            cp5.addSlider("stickSizeY", (val) => { stickSizeY = (float)val * scale * 0.1f; renewStick(); }, 1, 50, 5, 20, 540);
            cp5.addSlider("stickSizeZ", (val) => { stickSizeZ = (float)val * scale * 0.1f; renewStick(); }, 50, 500, 300, 20, 590);

            cp5.addSlider("releaseHt", (val) => { releaseHt = (float)val * scale*0.1f; renewHeight(); makeManyBoxes(); }, 0, 1500, 500, 20, 640);

            cp5.addButton("Run!", () => { run = !run; }, 20, 740);
            cp5.addButton("Export!", () => { export(); }, 120, 740);
            cp5.addButton("import path!", () => { import(); }, 220, 740);
            cp5.Show();
            gravity = 300 * scale;
            friction = 1f;
            //stick
            stickMass = 1 * scale;

            Freelook.SetEyeTarget(eye, target);
            this.tmpTarget = new Vector3(target.X,target.Y,target.Z);
            Graphics.SetFormText("BulletSharp - Benchmark Demo");
            Graphics.SetInfoText("Press R to start simulation\n" +
                                    "F3 - Toggle debug\n" +
                                    "Space - Shoot box\n" +
                                    "Total boxes:" + count);

        }

        ThreadSupportInterface CreateSolverThreadSupport(int maxNumThreads)
        {
            //#define SEQUENTIAL
            // if (SEQUENTIAL)
            /*{
                SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
                SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
                threadSupport->startSPU();
            }*/
            //else 
            // {

            Win32ThreadConstructionInfo threadConstructionInfo = new Win32ThreadConstructionInfo("solverThreads",
                Win32ThreadFunc.SolverThreadFunc, Win32LSMemorySetupFunc.SolverLSMemoryFunc, maxNumThreads);
            Win32ThreadSupport threadSupport = new Win32ThreadSupport(threadConstructionInfo);
            threadSupport.StartSpu();
             
            return threadSupport;
        }

        //Another setup()
        protected override void OnInitializePhysics()
        {
            // collision configuration contains default setup for memory, collision setup
            DefaultCollisionConstructionInfo cci = new DefaultCollisionConstructionInfo();
            cci.DefaultMaxPersistentManifoldPoolSize = 32768*10;
            cci.DefaultMaxCollisionAlgorithmPoolSize = 32768*10;
            CollisionConf = new DefaultCollisionConfiguration(cci);

            if (UseParallelDispatcherBenchmark)
            {
                int maxNumOutstandingTasks = 4;

                Win32ThreadConstructionInfo info = new Win32ThreadConstructionInfo("collision",
                    Win32ThreadFunc.ProcessCollisionTask, Win32LSMemorySetupFunc.CreateCollisionLocalStoreMemory,
                    maxNumOutstandingTasks);

                Win32ThreadSupport threadSupportCollision = new Win32ThreadSupport(info);
                Dispatcher = new SpuGatheringCollisionDispatcher(threadSupportCollision, 1, CollisionConf);
            }
            else
            {
                Dispatcher = new CollisionDispatcher(CollisionConf);
                //Dispatcher.DispatcherFlags = DispatcherFlags.DisableContactPoolDynamicAllocation;
            }

            // the maximum size of the collision world. Make sure objects stay within these boundaries
            // Don't make the world AABB size too large, it will harm simulation quality and performance
            Vector3 worldAabbMin = new Vector3(-1000, -1000, -1000);
            Vector3 worldAabbMax = new Vector3(1000, 1000, 1000);

            HashedOverlappingPairCache pairCache = new HashedOverlappingPairCache();
            Broadphase = new DbvtBroadphase(pairCache);
            //Broadphase = new DbvtBroadphase();
            if (UseParallelDispatcherBenchmark)
            {
                ThreadSupportInterface thread = CreateSolverThreadSupport(4);
                Solver = new ParallelConstraintSolver(thread);
            }
            else
            {
                Solver = new SequentialImpulseConstraintSolver();
            }
            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.Gravity = new Vector3(0, -(gravity),0);

            if (UseParallelDispatcherBenchmark)
            {
                ((DiscreteDynamicsWorld)World).SimulationIslandManager.SplitIslands = false;
            }
            World.SolverInfo.SolverMode |= SolverModes.EnableFrictionDirectionCaching;
            World.SolverInfo.NumIterations = 5;
            // create the ground
            CollisionShape plane = new StaticPlaneShape(new Vector3(0,1,0),0);
            CollisionShapes.Add(plane);
            CollisionObject ground = base.LocalCreateRigidBody(0, Matrix.Translation(0, 0, 0), plane);
            ground.UserObject = "Ground";

            //Create a dropping path
            import();
            renewHeight();

            //Create a base
            baseBoxes = new List<RigidBody>();
            renewBase();
            
            //Create a reference stick
            box = new BoxShape(stickSizeX/2f/2f, stickSizeZ/2f/2f, stickSizeY/2f/3f);
            stickSample = LocalCreateRigidBody(0, Matrix.Translation(new Vector3(-50,stickSizeZ/2f/2f,-30)), box);
            manyBoxes = new List<RigidBody>();
            makeManyBoxes();
            renewStick();
            World.PairCache.SetOverlapFilterCallback(new myFilterCallback());
        }
        void makeManyBoxes()
        {
            float margin=0.02f;
            BoxShape tinyBox = new BoxShape(0.2f - margin, 0.2f - margin, 0.2f - margin);
            /*for (int x = 0; x < 200; x++)
            {
                for (int y = 0; y < 200; y++)
                {
                    if (y < 50)
                    {
                        RigidBody tinyBoxBody = LocalCreateRigidBody(0, Matrix.Translation(new Vector3(x - 100, y + 5, 0)), tinyBox);
                    }
                    else
                    {
                        RigidBody tinyBoxBody = LocalCreateRigidBody(1, Matrix.Translation(new Vector3(x - 100, y + 5, 0)), tinyBox);
                    }
                    
                }
            }*/
            if (manyBoxes == null) return;
            foreach (var f in manyBoxes)
            {
                World.RemoveRigidBody(f);
            }
            foreach (var v in _pos)
            {
                var b = LocalCreateRigidBody(0, Matrix.Translation(v), tinyBox);
                manyBoxes.Add(b);
                b.UserObject = "ghost";
            }
        }
        
        class myFilterCallback : OverlapFilterCallback
        {
            public override bool NeedBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
            {
                bool collides = (proxy0.CollisionFilterGroup & proxy1.CollisionFilterMask) != 0;
                collides = collides && ((proxy1.CollisionFilterGroup & proxy0.CollisionFilterMask)!=0);

                RigidBody rb0=proxy0.ClientObject as RigidBody;
                RigidBody rb1=proxy1.ClientObject as RigidBody;
                //add some additional logic here that modified 'collides'
                if (rb0.UserObject as string == "ghost") collides = false;
                if (rb1.UserObject as string == "ghost") collides = false;
                if (rb0.UserObject as string == "static" && rb1.UserObject as string == "static") collides = false;
                if (rb0.UserObject as string == "static" && rb1.UserObject as string == "ground") collides = false;
                if (rb0.UserObject as string == "ground" && rb1.UserObject as string == "static") collides = false;
                return collides;
            }
        }
        void renewHeight()
        {

            int N = 10;
            if (points == null) return;
            _pos = new Vector3[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                _pos[i] = new Vector3((float)points[i][0] * scale, (float)points[i][2] * scale + releaseHt, (float)points[i][1] * scale);
            }

            pos = new List<Vector3>();//new Vector3[(points.Count - 1) * N];
            for (int i = 0; i < points.Count - 2; i++)
            {
                double x = points[i + 1][0] - points[i][0];
                double y = points[i + 1][1] - points[i][1];
                double z = points[i + 1][2] - points[i][2];
                double x2 = points[i + 2][0] - points[i + 1][0];
                double y2 = points[i + 2][1] - points[i + 1][1];
                double z2 = points[i + 2][2] - points[i + 1][2];
                double val = Math.Sqrt(x * x + y * y + z * z);
                double val2 = Math.Sqrt(x2 * x2 + y2 * y2 + z2 * z2);

                if (val < val2 * 1.5)
                {
                    for (int j = i * N; j < (i + 1) * N; j++)
                    {
                        int s = j - i * N;
                        pos.Add(new Vector3((float)(points[i + 1][0] * s / ((float)N) + points[i][0] * (N - s) / ((float)N)) * scale, (float)(points[i + 1][2] * s / ((float)N) + points[i][2] * (N - s) / ((float)N)) * scale+releaseHt, (float)(points[i + 1][1] * s / ((float)N) + points[i][1] * (N - s) / ((float)N)) * scale));
                    }
                }
            }
        }
        void renewStick()
        {
            if (box == null) return;
            box = new BoxShape(stickSizeX/2f, stickSizeZ/2f, stickSizeY/2f);
            stickSample.CollisionShape = box;
            stickSample.WorldTransform = Matrix.Translation(new Vector3(-50, stickSizeZ / 2f, -30));
        }
        void renewBase()
        {
            if (pos == null) return;
            if (baseBoxes == null) return;
            foreach (var f in baseBoxes)
            {
                World.RemoveRigidBody(f);
            }
            baseBoxes.Clear();
            BoxShape brickAD = null;
            BoxShape brickBC = null;
            int S = 70;
            for (int i = 0; i < S; i++)
            {
                int j = i + 1;
                int k = i + 2;
                //if (j == S) j = 0;
                Vector3 P = _pos[i];
                Vector3 Q = _pos[j];
                Vector3 R = _pos[k];
                P.Y = 0;
                Q.Y = 0;
                R.Y = 0;
                Matrix M=Matrix.Identity;
                M.M11 = 2 * (P - Q).X;
                M.M12 = 2 * (P - Q).Y;
                M.M13 = 2 * (P - Q).Z;
                M.M21 = 2 * (P - R).X;
                M.M22 = 2 * (P - R).Y;
                M.M23 = 2 * (P - R).Z;
                M.M31 = 0;
                M.M32 = 1;
                M.M33 = 0;
                Vector4 b = new Vector4(P.LengthSquared() - Q.LengthSquared(), P.LengthSquared() - R.LengthSquared(), 0, 1);
                Matrix m=Matrix.Invert(M);
                float cx = Vector4.Dot(m.get_Rows(0), b);
                float cy = Vector4.Dot(m.get_Rows(1), b);
                float cz = Vector4.Dot(m.get_Rows(2), b);
                Vector3 center = new Vector3(cx, cy, cz);
                Vector3 P2 = new Vector3(P.X - cx, P.Y - cy, P.Z - cz);
                Vector3 Q2 = new Vector3(Q.X - cx, Q.Y - cy, Q.Z - cz);
                Vector3 PQ=P-Q;
                P.Y = 0;
                Q.Y = 0;
                P2.Y = 0;
                Q2.Y = 0;
                Vector3 dP = Vector3.Normalize(P2) * baseD / 2f;
                Vector3 dQ = Vector3.Normalize(Q2) * baseD / 2f;
                Vector3 A = P + dP;
                Vector3 B = P - dP;
                Vector3 C = Q - dQ;
                Vector3 D = Q + dQ;
                brickAD = new BoxShape((A - D).Length() / 2f, baseH / 2f, baseWallThickness / 2f);
                brickBC = new BoxShape((B - C).Length() / 2f, baseH / 2f, baseWallThickness / 2f);
                PQ.Normalize();
                double theta=Math.Acos(PQ.X);
                if (PQ.Z > 0) theta = -theta;
                Matrix T;
                T=Matrix.RotationY((float)theta);
                T=T*Matrix.Translation((A+D)/2f);
                T = T * Matrix.Translation(0, baseH / 2f, 0);
                baseBoxes.Add(LocalCreateRigidBody(0, T, brickAD));
                T = Matrix.RotationY((float)theta);
                T = T*Matrix.Translation((B + C) / 2f);
                T = T * Matrix.Translation(0, baseH/2f, 0);
                baseBoxes.Add(LocalCreateRigidBody(0, T, brickBC));
            }
            
        }
        public override RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, null, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);
            rbInfo.Dispose();
            body.ContactProcessingThreshold = defaultContactProcessingThreshold;
            body.WorldTransform = startTransform;
            body.SetDamping(0.01f, 0.01f);
            World.AddRigidBody(body);
            return body;
        }
    }

    static class Program
    {
        [STAThread]
        static void Main()
        {
            using (Demo demo = new StickDemo())
            {
                LibraryManager.Initialize(demo);
            }
        }
    }
}
