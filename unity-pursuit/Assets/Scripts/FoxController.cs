using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FoxController : MonoBehaviour
{
    private Animator animator;
    private TargetController tc;

    public bool isMoving = false;
    public bool isRunning = false;

    [Min(0f)]
    public float walking2RunningBarrier = 1f;

    // Start is called before the first frame update
    void Start()
    {
        animator = GetComponentInChildren<Animator>();
        tc = GetComponent<TargetController>();
    }

    // Update is called once per frame
    void Update() => UpdateAnimation();

    void UpdateAnimation()
    {
        var velmag = tc.vel.magnitude;

        if (velmag == 0f) // is standing
        {
            isMoving = false;
            isRunning = false;
        } else if (velmag < walking2RunningBarrier) { // is walking
            isMoving = true;
            isRunning = false;
        } else { // is running
            isMoving = true;
            isRunning = true;
        }

        animator.SetBool("isMoving", isMoving);
        animator.SetBool("isRunning", isRunning);
    }
}
