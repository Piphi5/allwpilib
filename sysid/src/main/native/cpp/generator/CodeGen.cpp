// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generator/CodeGen.h"

#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <functional>
#include <algorithm>
#include <iterator>
#include <iostream>



using namespace sysid;

CodeGenerator::CodeGenerator(ProjectData data, const std::string& templatePath)
    : m_data(data), m_templateFile(templatePath) {
  if (!m_templateFile) {
    // Throw exception
  }
  m_commandMap["set_constants"] =  &CodeGenerator::SetConstants;
  m_commandMap["set_sides"] = &CodeGenerator::SetSides;
  m_commandMap["create_motors"] = &CodeGenerator::CreateMotors;
  m_commandMap["init_motors"] = &CodeGenerator::InitializeMotors;
  m_commandMap["init_gyros"] =  &CodeGenerator::InitializeGyros;
  m_commandMap["enable_motors"] = &CodeGenerator::EnableMotors;
  m_commandMap["disable_motors"] = &CodeGenerator::DisableMotors;
  m_commandMap["manual_control"] = &CodeGenerator::ManualControl;
}

void CodeGenerator::GenerateCode(const std::string& savePath) {
  std::string projectString;
  std::string line;
  m_outFile.open(savePath);
  
  if (m_outFile) {
    while(std::getline(m_templateFile, line))
    {
      std::size_t pos = line.find("%");
      if (pos != std::string::npos) {
        auto iter = m_commandMap.find(line.substr(pos + 1));
        if (iter == m_commandMap.end()) {
          // throw exception
          std::cout << "Not found" << std::endl;
          return;
        }

        (this->*(iter->second))();

      } else {
        m_outFile << line << std::endl;
      }
      
    }

    m_outFile.close();

  } else {
    // Throw exception
  }

}

void CodeGenerator::SetConstants() {
  if (m_data.left_motor_types[0] == "SparkMax") {
    m_outFile << "\t" << "static private int ENCODER_EDGES_PER_REV = " 
    << (m_data.epr / 4)  << ";" << std::endl;

    m_outFile << "\t" << "static private double encoderConstant = " 
    << (1 / m_data.gearing)  << ";" << std::endl;
  } else {
    m_outFile << "\t" << "static private double ENCODER_EDGES_PER_REV = " 
    << ((double) m_data.epr / 4)  << ";" << std::endl;

    m_outFile << "\t" << "static private double encoderConstant = " 
    << (1 / m_data.gearing) << " * (1 / ENCODER_EDGES_PER_REV)" << 
    ";" << std::endl;
  }

  m_outFile << "\t" << "static private int ENCODER_EPR = " 
  << m_data.epr << ";" << std::endl;

  m_outFile << "\t" << "static private int PIDIDX = 0;"
  << std::endl;
  
}

void CodeGenerator::SetSides() {
  if (!m_data.right_motor_ports.empty()) {
    m_outFile << "\tDifferentialDrive drive;" << std::endl;
  } else {
    if (m_data.left_motor_types[0] == "PWM") {
      m_outFile << "\tSpeedControllerGroup leaderMotor;" << std::endl;
    } else {
      m_outFile << "\t" << m_data.left_motor_types[0] << " leaderMotor;" << std::endl;
    }
  }
}

void CodeGenerator::SetupEncoders() {
  m_outFile << "\t\t\tif (side != Sides.FOLLOWER) {" << std::endl;

  if (m_data.encoder_type == "roboRIO") {
    m_outFile << "Encoder encoder;" << std::endl;
  } else if (m_data.encoder_type == "Built-In") {
    if (m_data.left_motor_types[0] == "SPARK MAX (Brushless)") {
      m_outFile << "\t\t\t\tCANEncoder encoder = motor.getEncoder();" << std::endl;
      
    } else if (m_data.left_motor_types[0] == "SPARK MAX (Brushed)") {
      m_outFile << "\t\t\t\tCANEncoder encoder = motor.getEncoder(EncoderType.kQuadrature, ENCODER_EDGES_PER_REV);"
      << std::endl;
    } else {
      m_outFile << "\t\t\t\tmotor.configSelectedFeedbackSensor(";
      if (m_data.left_motor_types[0] == "TalonFX") {
        m_outFile << "FeedbackDevice.IntegratedSensor";
      } else {
        m_outFile << "FeedbackDevice.QuadEncoder";
      }
      m_outFile << ", PIDIDX, 10);" << std::endl;
    }

  } else if (m_data.encoder_type == "CANCoder / Alternate") {
    if (
      m_data.left_motor_types[0] == "SPARK MAX (Brushless)" ||
      m_data.left_motor_types[0] == "SPARK MAX (Brushed)"
    ) {
      m_outFile << "\t\t\t\t"
      << "CANEncoder encoder = motor.getAlternateEncoder(AlternateEncoderType.kQuadrature, ENCODER_EDGES_PER_REV);"
      << std::endl;
    } else {
      // TODO Figure out Cancoder setup
    }

  } else {
    // TODO Throw exception
  }

  m_outFile << "\t\t\t}" << std::endl;
}

void CodeGenerator::SetupSuppliers(const std::string& side, bool encoder_inverted, int encoder_ports[2]) {
  std::string uppercase_side (side.size(), ' '); 
  std::transform(side.begin(), side.end(), uppercase_side.begin(), ::toupper);
  m_outFile << "\t\t\tcase " << uppercase_side << ":" << std::endl;
  if (m_data.encoder_type == "roboRIO") {
    m_outFile << "\t\t\t\tencoder = new Encoder(" << encoder_ports[0] << ", " 
    << encoder_ports[1] << ");" << std::endl;
    m_outFile << std::boolalpha << "\t\t\t\tencoder.setReverseDirection(" << encoder_inverted << ");"
    << std::endl;
    m_outFile << "\t\t\t\tencoder.setDistancePerPulse(encoderConstant);" << std::endl;
    m_outFile << "\t\t\t\t" << side << "EncoderPosition = encoder::getDistance;" << std::endl;
    m_outFile << "\t\t\t\t" << side << "EncoderRate = encoder::getRate;" << std::endl;
  } else if (
    m_data.left_motor_types[0] == "SPARK MAX (Brushless)" ||
    m_data.left_motor_types[0] == "SPARK MAX (Brushed)"
    ) {
    m_outFile << std::boolalpha << "\t\t\t\tencoder.setInverted(" << encoder_inverted << ");"
    << std::endl;
    m_outFile << side << "\t\t\t\tEncoderPosition () -> encoder.getPosition() * encoderConstant;"
    << std::endl;
    m_outFile << side << "\t\t\t\tEncoderRate = () -> encoder.getVelocity() * encoderConstant / 60.;"
    << std::endl;

  } else if (m_data.encoder_type == "Built-In") {

    m_outFile << std::boolalpha << "\t\t\t\tmotor.setSensorPhase(" << encoder_inverted << ");" 
    << std::endl;
    m_outFile << "\t\t\t\t" << side 
    << "EncoderPosition = () -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;"
    << std::endl;
    m_outFile << "\t\t\t\t" << side
    << "EncoderRate = () -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant * 10;"
    << std::endl;       

  } else if (m_data.encoder_type == "CANCoder / Alternate") {
    // TODO Figure out Canncoder                                           
    
  } else {
    // TODO Throw exception
  }

  m_outFile << "\t\t\t\tbreak;" << std::endl;


}

void CodeGenerator::CreateMotors() {
  // creates a copy of the motor controller names (assumes left and right side have same motor controllers)
  std::vector<std::string> all_motors (m_data.left_motor_types); 

  // get all unique items
  std::sort(all_motors.begin(), all_motors.end());
  std::vector<std::string>::iterator it;
  it = std::unique(all_motors.begin(), all_motors.end());  

  all_motors.resize(std::distance(all_motors.begin(),it));

  for (const std::string& motor : all_motors) {
    SetupMotor(motor);
  }

}

void CodeGenerator::SetupMotor(const std::string& motor) {
  // method header
  m_outFile << "\tpublic "<< motor << " setup" << motor
  << "(int port, Sides side, boolean inverted) {" << std::endl;

  // motor initialization
  if (motor != "SparkMax") {
    m_outFile << "\t\t" << motor << " motor = new " << motor << "(port);" << std::endl;
    if (motor != "PWM") {
      m_outFile << "\t\tmotor.configFactoryDefault();" << std::endl <<
      "\t\tmotor.setNeutralMode(NeutralMode.Brake);" << std::endl;
    }
  } else {
    m_outFile << "\t\tCANSparkMax motor = new CANSparkMax(port, ";
    if (motor == "SPARK MAX (Brushless)") {
      m_outFile << "MotorType.kBrushless";
    } else {
      m_outFile << "MotorType.kBrushed";
    }
    m_outFile << ");" << std::endl;

    m_outFile << "\t\tmotor.configFactoryDefault();" << std::endl;
    m_outFile << "\t\tmotor.setNeutralMode(NeutralMode.Brake);" << std::endl;
  }


  SetupEncoders();
  m_outFile << "\t\tswitch (side) {" << std::endl;
  SetupSuppliers("right", m_data.right_encoder_inverted, m_data.right_encoder_ports);
  SetupSuppliers("left", m_data.left_encoder_inverted, m_data.left_encoder_ports);
  m_outFile << "default:\n\t\t\t\tbreak;" << std::endl;
  m_outFile << "\t\t}" << std::endl;


  m_outFile << "\t}" << std::endl;
}

void CodeGenerator::ManualControl() {
  if (!m_data.right_motor_ports.empty()) {
    m_outFile << "\t\tdrive.arcadeDrive(-stick.getY(), stick.getX());" << std::endl;
  } else {
    m_outFile << "\t\tleaderMotor.set(-stick.getY());" << std::endl;
  }
}

void CodeGenerator::InitializeLeaders(const std::string& side, std::vector<std::string> motors, std::vector<int> ports, std::vector<bool> inverted) {
  std::string motor = motors[0];
  std::string uppercase_side (side.size(), ' '); 
  std::transform(side.begin(), side.end(), uppercase_side.begin(), ::toupper);
  m_outFile << "\t\t" << motor
  << side << "Motor = setup" << motor <<
  "(" << ports[0] 
  << ", Sides." << uppercase_side << ", " << inverted[0] << ");"
  << std::endl;
}
void CodeGenerator::InitializeFollowers(const std::string& side, std::vector<std::string> motors, std::vector<int> ports, std::vector<bool> inverted) {
  bool is_pwm = motors[0] == "PWM";
  if (is_pwm) {
    m_outFile << "ArrayList<SpeedController> " << side << "Motors = new ArrayList<SpeedController>();" << std::endl;
  }
  for (size_t i = 1; i < motors.size(); i++) {
    // initial indent
    m_outFile << "\t\t";
    if (is_pwm) {
      m_outFile << side << "Motors.add(setup" << m_data.left_motor_types[i]
      << "(" <<  m_data.left_motor_ports[i] << ", Sides.FOLLOWER, "
      << m_data.left_motor_inverted[i] << "));" << std::endl;
    } else {
      if (motors[i] == "SparkMax (Brushless)" || motors[i] == "SparkMax (Brushed)") {
        // initialize SparkMax Follower
        m_outFile << "CANSparkMax " << side << "Follower" << i << " = setupCANSparkMax("
        << ports[i] << "Sides.FOLLOWER, " << inverted[i] << ");" << std::endl;

        m_outFile << "\t\t" << side << "Follower" << i << ".follow(" 
        << side << "Motor, " << inverted[i] << ");" << std::endl;
      } else {
        // Initialize for CTRE Stuff
        m_outFile << side << "Motor" << i << " = setup" << motors[i] << "("
        << ports[i] << ", Sides.FOLLOWER, " << inverted[i] << ");" << std::endl;

        m_outFile << "\t\t" << side << "Motor.follow(" << side << "Motor);" << std::endl; 

      }
    }

    if (is_pwm) {
      m_outFile << "SpeedController[] " << side << "MotorControllers = new SpeedController[" 
      << side << "Motors.size()];" << std::endl;
      m_outFile << side << "motorControllers = " << side << "motors.toArray(" << side
      << "motorControllers);";
      m_outFile << "SpeedControllerGroup " << side << "Group = new SpeedControllerGroup("
      << side << "Motor, " << side << "MotorControllers);" << std::endl;
    }

  }
}
void CodeGenerator::InitializeMotors() {
  InitializeLeaders("left", m_data.left_motor_types, m_data.left_motor_ports, m_data.left_motor_inverted);
  InitializeFollowers("left", m_data.left_motor_types, m_data.left_motor_ports, m_data.left_motor_inverted);
  m_outFile << "\t\t";
  if (!m_data.right_motor_ports.empty()) {
    InitializeLeaders("right", m_data.right_motor_types, m_data.right_motor_ports, m_data.right_motor_inverted);
    InitializeFollowers("right", m_data.right_motor_types, m_data.right_motor_ports, m_data.right_motor_inverted);

    // create drivetrain setup
    m_outFile << "drive = new DifferentialDrive(leftGroup, rightGroup);" << std::endl;
  } else {
    // use left motors as main motor.
    if (m_data.left_motor_types[0] != "PWM") {
      m_outFile << "leaderMotor = leftMotor;" << std::endl;
    } else {
      m_outFile << "leaderMotor = leftGroup;" << std::endl;
    }
    
  }

}
void CodeGenerator::InitializeGyros() {
  if (m_data.gyro_type == "ADXRS450") {
    m_outFile << "\t\tGyro gyro = new ADXRS450_Gyro("
    << m_data.gyro_port << ");" << std::endl;

    m_outFile << "\t\tgyroAngleRadians = () -> -1 * Math.toRadians(gyro.getAngle());"
    << std::endl;
  } else if (m_data.gyro_type == "NavX") {
    m_outFile << "\t\tAHRS navx = new AHRS("
    << m_data.gyro_port << ");" << std::endl;

    m_outFile << "\t\tgyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());"
    << std::endl;
    
  } else if (m_data.gyro_type == "AnalogGyro") {
    m_outFile << "\t\tGyro gyro = new AnalogGyro("
    << m_data.gyro_port << ");" << std::endl;

    m_outFile << "\t\tgyroAngleRadians = () -> -1 * Math.toRadians(gyro.getAngle());"
    << std::endl;

  } else if (m_data.gyro_type == "Pigeon") {
    m_outFile << "\t\tPigeonIMU pigeon = new PigeonIMU(" 
    << m_data.gyro_port << ");" << std::endl;

    m_outFile << "\t\tgyroAngleRadians = () -> {\n"
    << "\t\t\tdouble[] xyz = new double[3];\n"
    << "\t\t\tpigeon.getAccumGyro(xyz);\n"
    << "\t\t\treturn Math.toRadians(xyz[2]);\n"
    << "\t\t};";

  } else {
    m_outFile << "\t\tgyroAngleRadians = () -> 0.0;" << std::endl;
  }
}
void CodeGenerator::EnableMotors() {
  if (!m_data.right_motor_ports.empty()) {
    m_outFile << "\t\tdrive.tankDrive("
    <<"(rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed," <<
    "false);" 
    << std::endl;
  } else {
    m_outFile << "\t\tleaderMotor.set(autospeed);" << std::endl;
  }
}
void CodeGenerator::DisableMotors() {
  
  if (!m_data.right_motor_ports.empty()) {
    m_outFile << "\t\tdrive.tankDrive(0, 0);" << std::endl;
  } else {
    m_outFile << "\t\tleaderMotor.set(0);" << std::endl;
  }
}