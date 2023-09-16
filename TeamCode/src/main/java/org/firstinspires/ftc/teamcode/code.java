@TeleOp(name="Robot: Ummdrive", group="drive")
public class RobotUmmdrive_Iterative extends OpMode{

    public DcMotor  motor1   = null;
    public DcMotor  motor2  = null;
    public DcMoter motor3 = null;
    public DcMoter motor4 = null;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
    }





    public
        @Override
        public void loop() {
        motor1.setPower(0.7);
        motor2.setPower(0.7);
        motor3.setPower(0.7);
        motor4.setPower(0.7);

            //fwrd = (1:f),(2:f),(3:f),(4:f)
            //leftspin = (1:b),(2:f),(3:b),(4:f)
            //rightspin = (1:f),(2:b),(3:f),(4:b)
            //back = (1:b),(2:b),(3:b),(4:b)
            //left = (1:b),(2:f),(3:f),(4:b)
            //right = (1:f),(2:b),(3:b),(4:f)

        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}