#ifndef __DEVICE_ICM42688_HPP__
#define __DEVICE_ICM42688_HPP__

class Icm42688 : public ImuAbstract
{
  public:
    Icm42688();
    ~Icm42688();

    int Init(void);
    int DeInit(void);
    int Read(void);

  private:
    int Process(void);
#ifdef CONFIG_ICM42688_TRIGGER
    int Trigger(void);
#endif /* CONFIG_ICM42688_TRIGGER */

  private:
    struct device* dev_;
#ifdef CONFIG_ICM42688_TRIGGER
    struct sensor_trigger trigger_;
#endif /* CONFIG_ICM42688_TRIGGER */
};

#endif // ! __DEVICE_ICM42688_HPP__
