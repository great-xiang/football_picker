using System.IO.MemoryMappedFiles;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Threading;
using System;

public class Main : MonoBehaviour
{
    float timer = 0;
    bool isScreenShot = false;
    float angle = 0;//偏向角
    float speed = 0;//总速度
    Transform m_Transform;
    int Done = 0;//Done表示抓球结果，Done=0没碰到任何东西，Done=1抓到球了，Done=2碰到障碍物嗝屁。
    int haswrite = 0;
    int trig = 0;
    int i = 0;



    private void Start()
    {
        m_Transform = gameObject.GetComponent<Transform>();

    }

    //unity每帧调用一次
    private void FixedUpdate()
    {
        timer += Time.deltaTime;
        m_Transform.Translate(Time.deltaTime * speed * Vector3.forward, Space.Self);//以自身坐标系向前
        //向python传递图片
        if (timer > 1)
        {
            //识别10次图像后结束本回合
            trig++;
            //if (trig == 10) { Done = 2; trig = 0; SceneManager.LoadScene(0); Debug.Log("重载游戏"); }
            timer = 0;
            isScreenShot = true;
            Time.timeScale = 0;//暂停游戏
        }
    }

    //该函数每次摄像机图像渲染后被调用，向python传输数据
    private void OnPostRender()
    {
        if (isScreenShot)
        {
            isScreenShot = false;
            //python读取数据的标志
            haswrite = (++haswrite) % 10;
            //读取unity摄像机图像
            Rect rect = new Rect(0, 0, Screen.width, Screen.height);
            Texture2D screenShot = new(224, 224, TextureFormat.RGB24, false);
            screenShot.ReadPixels(rect, 0, 0);
            screenShot.Apply();
            byte[] bytes = screenShot.EncodeToPNG();


            //WriteArray<T>(Int64, T[], Int32, Int32)用法
            //访问器中起始写入位置的字节偏移量。要写入访问器的数组。在 array 中从其开始写入的索引。要写入的 array 中的结构数。


            //创建名称为“img”，空间为100000的共享内存空间
            MemoryMappedFile img = MemoryMappedFile.CreateOrOpen("img", 100000);
            var viewAccessor0 = img.CreateViewAccessor(0, 100000);
            //写入结束信号Done
            viewAccessor0.WriteArray<byte>(0, System.Text.Encoding.Default.GetBytes(Done.ToString()), 0, 1);
            //写入图片尺寸
            string length = bytes.Length.ToString();
            viewAccessor0.WriteArray<byte>(1, System.Text.Encoding.Default.GetBytes(length), 0, 5);
            //写入图片数据
            viewAccessor0.WriteArray<byte>(6, bytes, 0, bytes.Length);

            //创建python读取标志
            MemoryMappedFile unitywrite;
            while(true)
            {
                try
                {
                    unitywrite = MemoryMappedFile.CreateOrOpen("unitywrite", 1);
                    var viewAccessor1 = unitywrite.CreateViewAccessor(0, 1);
                    viewAccessor1.WriteArray<byte>(0, System.Text.Encoding.Default.GetBytes("1"), 0, 1);
                    break;
                }
                catch { }
                finally { Thread.Sleep(10);}
            }


            //共享内存，读取python命令
            i = 0;
            while (true)
            {
                try { MemoryMappedFile pythonwrite = MemoryMappedFile.OpenExisting("pythonwrite");break; }
                catch { }
                finally { Thread.Sleep(10);i++; }
                if (i >= 1000) { Application.Quit(); break; }

            }


            MemoryMappedFile order;
            try
            {
                order = MemoryMappedFile.OpenExisting("order");
            }
            catch { Application.Quit(); }
            order = MemoryMappedFile.OpenExisting("order");
            MemoryMappedViewAccessor viewAccessor2 = order.CreateViewAccessor(0, 20);

            byte[] tempbytes = new byte[8];
            viewAccessor2.ReadArray<byte>(0, tempbytes, 0, 8);
            string tempstr = System.Text.Encoding.Default.GetString(tempbytes);
            angle = float.Parse(tempstr);

            viewAccessor2.ReadArray<byte>(8, tempbytes, 0, 8);
            tempstr = System.Text.Encoding.Default.GetString(tempbytes);
            speed = float.Parse(tempstr);

            byte tempbyte = viewAccessor2.ReadByte(16);
            tempstr = tempbyte.ToString();
            int python_Done = int.Parse(tempstr);

            //释放共享内存
            img.Dispose();
            unitywrite.Dispose();


            //机器人旋转
            m_Transform.Rotate(Vector3.up, angle);
            Debug.Log("旋转角度:" + angle + ",运动速度:" + speed);


            if (python_Done == 2) { Done = python_Done; SceneManager.LoadScene(0); Time.timeScale = 1; }
            else { Time.timeScale = 1; Done = 0; }

        };
    }


    //碰撞函数，碰到物体被调用，结束本回合，重新开始
    void OnCollisionEnter(Collision collision)
    {
        var name = collision.collider.name;
        Debug.Log("碰到了" + name);
        if (name == "football")
        {
            Done = 1;
        }
        else
        {
            Done = 2;
        }
        //发送结束信息
        isScreenShot = true;
        //暂停游戏
        Time.timeScale = 0;
        //重启unity
        SceneManager.LoadScene(0);
        Debug.Log("重载游戏");
    }
}


