// package team3647.lib.inputs;

// import java.sql.Driver;
// import java.util.ArrayList;
// import java.util.Collections;
// import java.util.List;

// import javax.swing.text.html.HTMLDocument.HTMLReader.FormAction;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.PS4Controller.Button;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import team3647.lib.team6328.VirtualSubsystem;

// public class ButtonHandler{
//     public static double cacheTimer = 0;
//     //public static List<ButtonPress> presses = List.of(new ButtonPress("default", 0));
//     public static ArrayList<ButtonPress> presses = new ArrayList<ButtonPress>(10);
//     public static ButtonHandler instance = new ButtonHandler();

//     public ButtonHandler(){
//         super();
//     }

//     public static void init(){
//         cacheTimer = Timer.getFPGATimestamp();
//     }

//     public Command addPress(ButtonPress press){
//         return Commands.runOnce(() -> presses.add(press));
//     }

//     public Command addPress(String name){
//         return Commands.runOnce(() -> presses.add(new ButtonPress(name,
// Timer.getFPGATimestamp())));
//     }

//     public Boolean getIsDoubleTap(ButtonPress press){

//         for(ButtonPress p : presses){
//             if(p.name.equals(press.name) && Math.abs(p.timestamp - press.timestamp) < 0.35){
//                 Logger.recordOutput("string", Math.abs(p.timestamp - press.timestamp));
//                 return true;

//             }
//         }
//         return false;
//     }

//     public void periodic() {
//         // if(Timer.getFPGATimestamp() - cacheTimer >= 2.0){
//         //     presses.clear();
//         //     cacheTimer = Timer.getFPGATimestamp();
//         // }

//       for(ButtonPress p : presses){
//         DriverStation.reportError(p.name, false);
//         DriverStation.reportError(String.valueOf(p.timestamp), false);
//         DriverStation.reportError("types hit", false);
//         DriverStation.reportError(getIsDoubleTap(p).toString(), false);

//       }
//       DriverStation.reportError("looping",false );

//     }

//     public static ButtonHandler getInstance(){
//         return instance;
//     }

// }
