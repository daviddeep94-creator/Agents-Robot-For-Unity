using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Foot : MonoBehaviour
{
    RobotBalanceAgent agent;
    bool isleft;
    private void Awake()
    {
        agent = GetComponentInParent<RobotBalanceAgent>();
        isleft = name.Contains("Left");
    }
    public void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.layer == 3)
        {
            agent.OnGround(isleft);
        }
    }
}
