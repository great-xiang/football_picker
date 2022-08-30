using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;
using System.Diagnostics;
using System.IO.MemoryMappedFiles;


/*
unity中的坐标是左手系
z是蓝箭头，表示前方
从上往下看，z方向顺时针旋转90是x方向，x方向是红箭头
y表示上方
旋转中数值y表示：从上向下看绕y轴顺时针旋转的角度（角度值）
*/


//输入z，x方向速度和运动时间。物体先转向后运动
public class turn_move : MonoBehaviour
{
    float timer = 0;
    bool isScreenShot = false;
    public float vx=1,vz=1;//二维运动变量
    private float v_sum;//总速度
    private float turn_radian;//旋转角度（-pi， pi]
    private Transform m_Transform;

    //保存图片的函数
    private void OnPostRender()
    {
        if (isScreenShot)
        {
            Rect rect = new Rect(0, 0, Screen.width, Screen.height);
            // 先创建一个的空纹理，大小可根据实现需要来设置
            Texture2D screenShot = new Texture2D(224,224, TextureFormat.RGB24, false);

            // 读取屏幕像素信息并存储为纹理数据，
            screenShot.ReadPixels(rect, 0, 0);
            screenShot.Apply();
            byte[] bytes = screenShot.EncodeToPNG();

            //File.WriteAllBytes(Application.dataPath + $"/screen_capture/onPcSavedScreen.jpg", bytes);
            long space = 24 * 24 * 3*3*100;
            var mmf = MemoryMappedFile.CreateOrOpen("addr", space);//内存映射文件名，空间大小
            var viewAccessor = mmf.CreateViewAccessor(0, space);//创建映射到内存映射文件视图的可随机访问的内存块.
            viewAccessor.Write(0, bytes.Length);
            //访问器中起始写入位置的字节偏移量。要写入访问器的数组。在 array 中从其开始写入的索引。要写入的 array 中的结构数。
            viewAccessor.WriteArray<byte>(0, bytes, 0, bytes.Length);
            RunPythonScript();
            Destroy(screenShot);
            Time.timeScale = 1;//游戏继续
            isScreenShot = false;
        }
    }

    // 旋转函数
    void Start()
    {
        m_Transform = gameObject.GetComponent<Transform>();
        v_sum = (float)System.Math.Sqrt(vx * vx + vz * vz);

        if (vx > 0) {//顺时针旋转（0，pi）
            turn_radian = Mathf.Acos(vz / v_sum);//反余弦函数
        }
        else if (vx == 0 && vz < 0) {//顺时针旋转pi
            turn_radian = Mathf.PI;
            //m_Transform.Rotate(Vector3.up, 180);
        }
        else if (vx < 0) {//顺时针旋转（-pi， 0]       
            turn_radian = - 1 *Mathf.PI + Mathf.Acos((vz * -1) / v_sum);
        }

        m_Transform.Rotate(Vector3.up, turn_radian * 180 / Mathf.PI);//开始旋转，弧度换角度
    }

    //游戏更新
    void FixedUpdate()
    {
        timer += Time.deltaTime;
        m_Transform.Translate(Vector3.forward * Time.deltaTime * v_sum, Space.Self);//以自身坐标系向前
        if (timer > 0.5) {
            timer = 0;
            isScreenShot = true;
            Time.timeScale = 0;//暂停游戏
        }
    }
    //碰到足球后结束游戏，重新开始
     void OnCollisionEnter(Collision collision)
    {
        var name = collision.collider.name;
        UnityEngine.Debug.Log("Name is " + name);
        if (name == "football") {

        };
        if (name == "people")
        {

        };
        SceneManager.LoadScene(0);
    }


    //调用python代码
    public static void RunPythonScript()
    {
        Process p = new Process();
        string path = @"C:\Mycode\yolox-pytorch-main\unity.py";
        path += path;
        p.StartInfo.FileName = @"C:\Python38\python.exe";
        p.StartInfo.Arguments = path;
        p.StartInfo.UseShellExecute = false;
        p.StartInfo.RedirectStandardOutput = true;
        p.StartInfo.RedirectStandardInput = true;
        p.StartInfo.RedirectStandardError = true;
        p.StartInfo.CreateNoWindow = true;

        p.Start();
        p.BeginOutputReadLine();
        p.OutputDataReceived += new DataReceivedEventHandler(Get_data);
        p.WaitForExit();
    }
    private static void Get_data(object sender, DataReceivedEventArgs eventArgs)
    {
        if (!string.IsNullOrEmpty(eventArgs.Data))
        {
            print(eventArgs.Data);
        }
    }

}

