using System.IO.MemoryMappedFiles;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Threading;
using System;

public class Main : MonoBehaviour
{
    float timer = 0;
    bool isScreenShot = false;
    float angle = 0;//ƫ���
    float speed = 0;//���ٶ�
    Transform m_Transform;
    int Done = 0;//Done��ʾץ������Done=0û�����κζ�����Done=1ץ�����ˣ�Done=2�����ϰ�����ƨ��
    int haswrite = 0;
    int trig = 0;
    int i = 0;



    private void Start()
    {
        m_Transform = gameObject.GetComponent<Transform>();

    }

    //unityÿ֡����һ��
    private void FixedUpdate()
    {
        timer += Time.deltaTime;
        m_Transform.Translate(Time.deltaTime * speed * Vector3.forward, Space.Self);//����������ϵ��ǰ
        //��python����ͼƬ
        if (timer > 1)
        {
            //ʶ��10��ͼ���������غ�
            trig++;
            //if (trig == 10) { Done = 2; trig = 0; SceneManager.LoadScene(0); Debug.Log("������Ϸ"); }
            timer = 0;
            isScreenShot = true;
            Time.timeScale = 0;//��ͣ��Ϸ
        }
    }

    //�ú���ÿ�������ͼ����Ⱦ�󱻵��ã���python��������
    private void OnPostRender()
    {
        if (isScreenShot)
        {
            isScreenShot = false;
            //python��ȡ���ݵı�־
            haswrite = (++haswrite) % 10;
            //��ȡunity�����ͼ��
            Rect rect = new Rect(0, 0, Screen.width, Screen.height);
            Texture2D screenShot = new(224, 224, TextureFormat.RGB24, false);
            screenShot.ReadPixels(rect, 0, 0);
            screenShot.Apply();
            byte[] bytes = screenShot.EncodeToPNG();


            //WriteArray<T>(Int64, T[], Int32, Int32)�÷�
            //����������ʼд��λ�õ��ֽ�ƫ������Ҫд������������顣�� array �д��俪ʼд���������Ҫд��� array �еĽṹ����


            //��������Ϊ��img�����ռ�Ϊ100000�Ĺ����ڴ�ռ�
            MemoryMappedFile img = MemoryMappedFile.CreateOrOpen("img", 100000);
            var viewAccessor0 = img.CreateViewAccessor(0, 100000);
            //д������ź�Done
            viewAccessor0.WriteArray<byte>(0, System.Text.Encoding.Default.GetBytes(Done.ToString()), 0, 1);
            //д��ͼƬ�ߴ�
            string length = bytes.Length.ToString();
            viewAccessor0.WriteArray<byte>(1, System.Text.Encoding.Default.GetBytes(length), 0, 5);
            //д��ͼƬ����
            viewAccessor0.WriteArray<byte>(6, bytes, 0, bytes.Length);

            //����python��ȡ��־
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


            //�����ڴ棬��ȡpython����
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

            //�ͷŹ����ڴ�
            img.Dispose();
            unitywrite.Dispose();


            //��������ת
            m_Transform.Rotate(Vector3.up, angle);
            Debug.Log("��ת�Ƕ�:" + angle + ",�˶��ٶ�:" + speed);


            if (python_Done == 2) { Done = python_Done; SceneManager.LoadScene(0); Time.timeScale = 1; }
            else { Time.timeScale = 1; Done = 0; }

        };
    }


    //��ײ�������������屻���ã��������غϣ����¿�ʼ
    void OnCollisionEnter(Collision collision)
    {
        var name = collision.collider.name;
        Debug.Log("������" + name);
        if (name == "football")
        {
            Done = 1;
        }
        else
        {
            Done = 2;
        }
        //���ͽ�����Ϣ
        isScreenShot = true;
        //��ͣ��Ϸ
        Time.timeScale = 0;
        //����unity
        SceneManager.LoadScene(0);
        Debug.Log("������Ϸ");
    }
}


