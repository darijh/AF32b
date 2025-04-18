#ifndef PERIODIC_TASK_H
#define PERIODIC_TASK_H

class PeriodicTask
{
public:
    // Constructor que recibe el intervalo en milisegundos
    PeriodicTask(unsigned long intervalMs)
        : interval(intervalMs), lastExecution(0), cb(nullptr) {}

    // Método que debe llamarse periódicamente (por ejemplo desde loop())
    void run(unsigned long currentMillis)
    {
        if (cb && (currentMillis - lastExecution >= interval))
        {
            lastExecution = currentMillis;
            cb();
        }
    }

    // Establece la función callback que se ejecutará periódicamente
    void setCallback(std::function<void()> newCallback)
    {
        cb = newCallback;
    }

private:
    unsigned long interval;
    std::function<void()> cb;
    unsigned long lastExecution;
};

#endif // PERIODIC_TASK_H