using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;
using System.Diagnostics;
using System.IO.MemoryMappedFiles;


/*
unity�е�����������ϵ
z������ͷ����ʾǰ��
�������¿���z����˳ʱ����ת90��x����x�����Ǻ��ͷ
y��ʾ�Ϸ�
��ת����ֵy��ʾ���������¿���y��˳ʱ����ת�ĽǶȣ��Ƕ�ֵ��
*/


//����z��x�����ٶȺ��˶�ʱ�䡣������ת����˶�
public class turn_move : MonoBehaviour
{
    float timer = 0;
    bool isScreenShot = false;
    public float vx=1,vz=1;//��ά�˶�����
    private float v_sum;//���ٶ�
    private float turn_radian;//��ת�Ƕȣ�-pi�� pi]
    private Transform m_Transform;

    //����ͼƬ�ĺ���
    private void OnPostRender()
    {
        if (isScreenShot)
        {
            Rect rect = new Rect(0, 0, Screen.width, Screen.height);
            // �ȴ���һ���Ŀ�������С�ɸ���ʵ����Ҫ������
            Texture2D screenShot = new Texture2D(224,224, TextureFormat.RGB24, false);

            // ��ȡ��Ļ������Ϣ���洢Ϊ�������ݣ�
            screenShot.ReadPixels(rect, 0, 0);
            screenShot.Apply();
            byte[] bytes = screenShot.EncodeToPNG();

            //File.WriteAllBytes(Application.dataPath + $"/screen_capture/onPcSavedScreen.jpg", bytes);
            long space = 24 * 24 * 3*3*100;
            var mmf = MemoryMappedFile.CreateOrOpen("addr", space);//�ڴ�ӳ���ļ������ռ��С
            var viewAccessor = mmf.CreateViewAccessor(0, space);//����ӳ�䵽�ڴ�ӳ���ļ���ͼ�Ŀ�������ʵ��ڴ��.
            viewAccessor.Write(0, bytes.Length);
            //����������ʼд��λ�õ��ֽ�ƫ������Ҫд������������顣�� array �д��俪ʼд���������Ҫд��� array �еĽṹ����
            viewAccessor.WriteArray<byte>(0, bytes, 0, bytes.Length);
            RunPythonScript();
            Destroy(screenShot);
            Time.timeScale = 1;//��Ϸ����
            isScreenShot = false;
        }
    }

    // ��ת����
    void Start()
    {
        m_Transform = gameObject.GetComponent<Transform>();
        v_sum = (float)System.Math.Sqrt(vx * vx + vz * vz);

        if (vx > 0) {//˳ʱ����ת��0��pi��
            turn_radian = Mathf.Acos(vz / v_sum);//�����Һ���
        }
        else if (vx == 0 && vz < 0) {//˳ʱ����תpi
            turn_radian = Mathf.PI;
            //m_Transform.Rotate(Vector3.up, 180);
        }
        else if (vx < 0) {//˳ʱ����ת��-pi�� 0]       
            turn_radian = - 1 *Mathf.PI + Mathf.Acos((vz * -1) / v_sum);
        }

        m_Transform.Rotate(Vector3.up, turn_radian * 180 / Mathf.PI);//��ʼ��ת�����Ȼ��Ƕ�
    }

    //��Ϸ����
    void FixedUpdate()
    {
        timer += Time.deltaTime;
        m_Transform.Translate(Vector3.forward * Time.deltaTime * v_sum, Space.Self);//����������ϵ��ǰ
        if (timer > 0.5) {
            timer = 0;
            isScreenShot = true;
            Time.timeScale = 0;//��ͣ��Ϸ
        }
    }
    //��������������Ϸ�����¿�ʼ
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


    //����python����
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

