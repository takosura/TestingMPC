using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{
    public float SpringConst;
    public float NaturalLength;
    public GameObject InputObject;
    public GameObject Object;
    public GameObject Spring;
    Transform InputObjectTransform;
    Transform ObjectTransform;
    Transform SpringTransform;
    Rigidbody ObjectBody;
    // Start is called before the first frame update
    void Start()
    {
        InputObjectTransform=InputObject.GetComponent<Transform>();   
        ObjectTransform=Object.GetComponent<Transform>();   
        SpringTransform=Spring.GetComponent<Transform>();   
        ObjectBody=Object.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        float diff=InputObjectTransform.position.y-ObjectTransform.position.y;
        float SpringPosition_y=InputObjectTransform.position.y-diff/2;
        SpringTransform.position=new Vector3(0,SpringPosition_y,0);
        SpringTransform.localScale=new Vector3(0.05f,diff/2,0.05f);
        ObjectBody.AddForce(0,(diff-NaturalLength)*SpringConst,0);
    }
}
