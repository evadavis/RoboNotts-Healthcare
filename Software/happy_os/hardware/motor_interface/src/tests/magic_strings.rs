use crate::message::{ModbusCommand, ModbusRegister, ModbusRequest};

const MOTOR_GET_VELOCITY_MAGIC_FRAME: [u8; 6] = [0x01, 0x03, 0x00, 0x10, 0x0, 0x1];
const MOTOR_SET_VELOCITY_MAGIC_FRAME: [u8; 6] = [0x01, 0x06, 0x00, 0x02, 0x0, 0x0];
const MOTOR_GET_POSITION_L_MAGIC_FRAME: [u8; 6] = [0x01, 0x03, 0x0, 0x16, 0x0, 0x01];
const MOTOR_GET_POSITION_H_MAGIC_FRAME: [u8; 6] = [0x01, 0x03, 0x0, 0x17, 0x0, 0x01];
const MOTOR_SET_POSITION_MAGIC_FRAME: [u8; 11] =
    [0x01, 0x10, 0x0, 0x0c, 0x0, 0x02, 0x04, 0x0, 0x0, 0x0, 0x0];
const MOTOR_GET_STATUS_MAGIC_FRAME: [u8; 6] = [0x01, 0x03, 0x0, 0x0E, 0x0, 0x01];
const MOTOR_SET_POSITION_GAIN_MAGIC_FRAME: [u8; 6] = [0x01, 0x06, 0x00, 0x07, 0x0, 0x0];
const MOTOR_SET_POSITION_FF_MAGIC_FRAME: [u8; 6] = [0x01, 0x06, 0x00, 0x19, 0x0, 0x0];

const MOTOR_DISABLE_MAGIC_FRAME: [u8; 8] = [0x01, 0x06, 0x0, 0x1, 0x0, 0x0, 0xd8, 0x0a];
const MOTOR_ENABLE_MAGIC_FRAME: [u8; 8] = [0x01, 0x06, 0x0, 0x1, 0x0, 0x01, 0x19, 0xca];
const MODBUS_ENABLE_MAGIC_FRAME : [u8; 8] = [0x01, 0x06, 0x0, 0x0, 0x0, 0x01, 0x48, 0x0a];

#[test]
fn check_motor_enable() {
    let sut = ModbusRequest {
        device_address: 0x1,
        command: ModbusCommand::WriteRegister,
        register: ModbusRegister::EnableMotor,
        value: 0x1,
    };

    assert_eq!(MOTOR_ENABLE_MAGIC_FRAME, sut.to_message_bytes())
}

#[test]
fn check_modbus_enable() {
    let sut = ModbusRequest {
        device_address: 0x1,
        command: ModbusCommand::WriteRegister,
        register: ModbusRegister::EnableModbus,
        value: 0x1,
    };

    assert_eq!(MODBUS_ENABLE_MAGIC_FRAME, sut.to_message_bytes())
}

#[test]
fn check_motor_disable() {
    let sut = ModbusRequest {
        device_address: 0x1,
        command: ModbusCommand::WriteRegister,
        register: ModbusRegister::EnableMotor,
        value: 0x0,
    };

    assert_eq!(MOTOR_DISABLE_MAGIC_FRAME, sut.to_message_bytes())
}

#[test]
fn check_motor_get_position_low() {
    let sut = ModbusRequest {
        device_address: 0x1,
        command: ModbusCommand::ReadRegister,
        register: ModbusRegister::MotorAbsolutePositionLow,
        value: 0x1,
    };

    assert_eq!(MOTOR_GET_POSITION_L_MAGIC_FRAME, sut.to_message_bytes()[..6]);
}

#[test]
fn check_motor_get_position_high() {
    let sut = ModbusRequest {
        device_address: 0x1,
        command: ModbusCommand::ReadRegister,
        register: ModbusRegister::MotorAbsolutePositionHigh,
        value: 0x1,
    };

    assert_eq!(MOTOR_GET_POSITION_H_MAGIC_FRAME, sut.to_message_bytes()[..6]);
}