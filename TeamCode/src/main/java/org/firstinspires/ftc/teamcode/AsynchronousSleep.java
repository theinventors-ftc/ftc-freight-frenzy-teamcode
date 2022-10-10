package org.firstinspires.ftc.teamcode;

public class AsynchronousSleep {
    private boolean waits = false;
    private long start_time = -1;
    private long ms = -1;

    void AsynchronousSleep() {}

    public void wait(int ms)
    {
        if (!waits) {
            start_time = System.currentTimeMillis();
            waits = true;
            this.ms = ms;
        }
    }

    public void update()
    {
        if (System.currentTimeMillis() - start_time > ms) {
            waits = false;
        }
    }

    public boolean is_ready()
    {
        return !waits && start_time > 0;
    }

    public void reset()
    {
        waits = false;
        start_time = -1;
    }

    public boolean is_busy()
    {
        return waits;
    }
}
