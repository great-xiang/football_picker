using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Camera_move : MonoBehaviour
{

    private Transform m_Transform;

    // Start is called before the first frame update
    void Start()
    {
        m_Transform = gameObject.GetComponent<Transform>();
    }

    // Update is called once per frame
    void Update()
    {
        //wads空格，左ctrl 用来移动
        if (Input.GetKey(KeyCode.W)) { transform.Translate(Vector3.forward * 0.1f, Space.Self); }
        if (Input.GetKey(KeyCode.S)) { transform.Translate(Vector3.back * 0.1f, Space.Self); }
        if (Input.GetKey(KeyCode.A)) { transform.Translate(Vector3.left * 0.1f, Space.Self); }
        if (Input.GetKey(KeyCode.D)) { transform.Translate(Vector3.right * 0.1f, Space.Self); }
        if (Input.GetKey(KeyCode.Space)) { transform.Translate(Vector3.up * 00.1f, Space.Self); }
        if (Input.GetKey(KeyCode.LeftControl)) { transform.Translate(Vector3.down * 00.1f, Space.Self); }

        //ijlk 用来转向
        if (Input.GetKey(KeyCode.I)) { transform.Rotate(-1 * 0.1f, 0, 0); }
        if (Input.GetKey(KeyCode.K)) { transform.Rotate(1 * 0.1f, 0, 0); }
        if (Input.GetKey(KeyCode.J)) { transform.Rotate(0, -1 * 0.1f, 0); }
        if (Input.GetKey(KeyCode.L)) { transform.Rotate(0, 1 * 0.1f, 0); }
    }
}
