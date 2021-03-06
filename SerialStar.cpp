/* "Системы модули и компоненты" ("СМК"). 2020. Москва.
Библиотека C++ для модулей MBee .
Распространяется свободно. Надеемся, что программные продукты, созданные
с помощью данной библиотеки будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library.
Copyright © 2020 Systems, modules and components. Moscow. Russia.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Code adapted from  XBee-Arduino library XBee.h. Copyright info below.
* @file       XBee.h
* @author     Andrew Rapp
* @license    This project is released under the GNU License
* @copyright  Copyright (c) 2009 Andrew Rapp. All rights reserved
* @date       2009
* @brief      Interface to the wireless XBee modules
*/

#include "includes/MBee.h"

/*********************************************************************
* Public методы класса MBeeResponse.
*********************************************************************/
MBeeResponse::MBeeResponse()
{

}

uint8_t MBeeResponse::getApiId()
{
    return _apiId;
}

void MBeeResponse::setApiId(uint8_t apiId)
{
    _apiId = apiId;
}

uint8_t MBeeResponse::getMsbLength()
{
    return _msbLength;
}

void MBeeResponse::setMsbLength(uint8_t msbLength)
{
    _msbLength = msbLength;
}

uint8_t MBeeResponse::getLsbLength()
{
    return _lsbLength;
}

void MBeeResponse::setLsbLength(uint8_t lsbLength)
{
    _lsbLength = lsbLength;
}

uint8_t MBeeResponse::getChecksum()
{
    return _checksum;
}

void MBeeResponse::setChecksum(uint8_t checksum)
{
    _checksum = checksum;
}

uint8_t MBeeResponse::getFrameDataLength()
{
    return _frameLength;
}

void MBeeResponse::setFrameLength(uint8_t frameLength)
{
    _frameLength = frameLength;
}

void MBeeResponse::setFrameData(uint8_t* frameDataPtr)
{
    _frameDataPtr = frameDataPtr;
}

uint8_t* MBeeResponse::getFrameData()
{
    return _frameDataPtr;
}

uint16_t MBeeResponse::getPacketLength()
{
    return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
}

void MBeeResponse::setExtendedFieldsLength(uint8_t length)
{
  _extendedFieldsLength = length;
}

uint8_t MBeeResponse::getExtendedFieldsLength()
{
  return _extendedFieldsLength;
}

void MBeeResponse::reset()
{
    init();
    _apiId = 0;
    _msbLength = 0;
    _lsbLength = 0;
    _frameLength = 0;
    _extendedFieldsLength = 0;
}

void MBeeResponse::init()
{
    _complete = false;
    _errorCode = NO_ERROR_IN_FRAME;
    _checksum = 0;
}

void MBeeResponse::getTxStatusResponse(MBeeResponse &txResponse)
{
    TxStatusResponse* txStatus = static_cast<TxStatusResponse*>(&txResponse);
    txStatus->setFrameData(getFrameData());
    setCommon(txResponse);
}

void MBeeResponse::getRxResponse(MBeeResponse &rxResponse)
{
    RxResponse* rx = static_cast<RxResponse*>(&rxResponse);
    rx->setFrameData(getFrameData());
    setCommon(rxResponse);
}

void MBeeResponse::getRxIoSampleResponse(MBeeResponse &ioSample)
{
    RxIoSampleResponse* io = static_cast<RxIoSampleResponse*>(&ioSample);
    io->setFrameData(getFrameData());
    setCommon(ioSample);
}

void MBeeResponse::getModemStatusResponse(MBeeResponse &modemStatusResponse)
{
    ModemStatusResponse* modem = static_cast<ModemStatusResponse*>(&modemStatusResponse);
    modem->setFrameData(getFrameData());
    setCommon(modemStatusResponse);
}

void MBeeResponse::getAtCommandResponse(MBeeResponse &atCommandResponse)
{
    AtCommandResponse* at = static_cast<AtCommandResponse*>(&atCommandResponse);
    at->setFrameData(getFrameData());
    setCommon(atCommandResponse);
}

void MBeeResponse::getRemoteAtCommandResponse(MBeeResponse &response)
{
    RemoteAtCommandResponse* at = static_cast<RemoteAtCommandResponse*>(&response);
    at->setFrameData(getFrameData());
    setCommon(response);
}

void MBeeResponse::getRxAcknowledgeResponse(MBeeResponse &acknowledge)
{
    RxAcknowledgeResponse* ack = static_cast<RxAcknowledgeResponse*>(&acknowledge);
    ack->setFrameData(getFrameData());
    setCommon(acknowledge);
}

bool MBeeResponse::isAvailable()
{
    return _complete;
}

void MBeeResponse::setAvailable(bool complete)
{
    _complete = complete;
}

bool MBeeResponse::isError()
{
    return _errorCode > 0;
}

uint8_t MBeeResponse::getErrorCode()
{
    return _errorCode;
}

void MBeeResponse::setErrorCode(uint8_t errorCode)
{
    _errorCode = errorCode;
}

/*********************************************************************
* Private методы класса MBeeResponse.
*********************************************************************/
//Копирование общих полей из принятого сообщения.
void MBeeResponse::setCommon(MBeeResponse &target)
{
    target.setApiId(getApiId());
    target.setAvailable(isAvailable());
    target.setChecksum(getChecksum());
    target.setErrorCode(getErrorCode());
    target.setFrameLength(getFrameDataLength());
    target.setMsbLength(getMsbLength());
    target.setLsbLength(getLsbLength());
    target.setExtendedFieldsLength(0);
    if((getApiId() == RECEIVE_PACKET_EXTENDED_API_FRAME)  ||\
       (getApiId() == RECEIVE_PACKET_NO_OPTIONS_EXTENDED_API_FRAME) ||\
       (getApiId() == REMOTE_AT_COMMAND_RESPONSE_EXTENDED_API_FRAME) ||\
       (getApiId() == IO_DATA_SAMPLE_EXTENDED_API_FRAME))

    {
      target.setExtendedFieldsLength(3);
    }
}

/*********************************************************************
* Public методы класса FrameIdResponse.
*********************************************************************/
FrameIdResponse::FrameIdResponse()
{

}

uint8_t FrameIdResponse::getFrameId()
{
    return getFrameData()[0];
}

/*********************************************************************
* Public методы класса ModemStatusResponse.
*********************************************************************/
ModemStatusResponse::ModemStatusResponse()
{

}

uint8_t ModemStatusResponse::getStatus()
{
    return getFrameData()[0];
}

/*********************************************************************
* Public методы класса RxCommonResponse.
*********************************************************************/
RxCommonResponse::RxCommonResponse() : MBeeResponse()
{

}

uint16_t RxCommonResponse::getRemoteAddress()
{
    return(getFrameData()[0] << 8) + getFrameData()[1];
}

uint8_t RxCommonResponse::getRssi()
{
    return getFrameData()[2];
}

uint8_t RxCommonResponse::getOption()
{
    return getFrameData()[3];
}

uint8_t RxCommonResponse::getFrameId()
{
  if(getExtendedFieldsLength())
  {
    return getFrameData()[4];
  }
  return 0;
}

uint16_t RxCommonResponse::getPreviousHopAddress()
{
  if(getExtendedFieldsLength())
  {
    return(getFrameData()[5] << 8) + getFrameData()[6];
  }
  return 0;
}

/*********************************************************************
* Public методы класса AtCommandResponse.
*********************************************************************/
AtCommandResponse::AtCommandResponse()
{

}

uint8_t* AtCommandResponse::getCommand()
{
    return getFrameData() + 1;
}

uint8_t AtCommandResponse::getStatus()
{
    return getFrameData()[3];
}

uint8_t AtCommandResponse::getValueLength()
{
    return getFrameDataLength() - 4;
}

uint8_t* AtCommandResponse::getValue()
{
    if(getValueLength() > 0)
    {
        return getFrameData() + 4; //Поле параметра включается только для команд запроса. Команды устанвоки не имеют в ответе поля параметра.
    }
    return NULL;
}

bool AtCommandResponse::isOk()
{
    return getStatus() == OK_COMMAND_STATUS;
}

/*********************************************************************
* Public методы класса TxStatusResponse.
*********************************************************************/
TxStatusResponse::TxStatusResponse() : FrameIdResponse()
{

}

uint8_t TxStatusResponse::getStatus()
{
    return getFrameData()[4];
}

bool TxStatusResponse::isSuccess()
{
    return getStatus() == SUCCESS;
}

/*********************************************************************
* Public методы класса RemoteAtCommandResponse.
*********************************************************************/
RemoteAtCommandResponse::RemoteAtCommandResponse() : RxCommonResponse()
{

}

uint8_t* RemoteAtCommandResponse::getCommand()
{
  return getFrameData() + 4 + getExtendedFieldsLength();
}

uint8_t RemoteAtCommandResponse::getStatus()
{
  return getFrameData()[6 + getExtendedFieldsLength()];
}

bool RemoteAtCommandResponse::isOk()
{
    return getStatus() == OK_COMMAND_STATUS;
}

uint8_t RemoteAtCommandResponse::getValueLength()
{
    return getFrameDataLength() - 7 - getExtendedFieldsLength();
}

uint8_t* RemoteAtCommandResponse::getValue()
{
    if(getValueLength() > 0)
    {
        return getFrameData() + 7 + getExtendedFieldsLength();
    }
    return NULL;
}

/*********************************************************************
* Public методы класса RxAcknowledgeResponse.
*********************************************************************/
RxAcknowledgeResponse::RxAcknowledgeResponse() : RxCommonResponse()
{

}
uint8_t RxAcknowledgeResponse::getFrameId()
{
    return getFrameDataLength() - 4;
}

/*********************************************************************
* Public методы класса RxDataResponse.
*********************************************************************/
RxDataResponse::RxDataResponse() : RxCommonResponse()
{

}

uint8_t RxDataResponse::getData(uint8_t index)
{
    return getFrameData()[getDataOffset() + index];
}

uint8_t* RxDataResponse::getData()
{
    return getFrameData() + getDataOffset();
}

bool RxDataResponse::isAddressBroadcast()
{
    return (getOption() & BV(PACKET_WAS_BROADCAST_BIT));
}

/*********************************************************************
* Public методы класса RxIoSampleResponse.
*********************************************************************/
RxIoSampleResponse::RxIoSampleResponse() : RxDataResponse()
{

}
uint8_t RxIoSampleResponse::getDataLength()
{
    return getPacketLength() - 6 - getExtendedFieldsLength(); //Считаем, что поле данных начинается после полей температуры и напряжения на батарее.
}

uint8_t RxIoSampleResponse::getDataOffset()
{
    return 6 + getExtendedFieldsLength();
}

uint8_t RxIoSampleResponse::getSampleSize()
{
    uint8_t count = 0;
    uint8_t i = 1;
    uint8_t & iRef = i;
    while(i < getDataLength())
    {
        getNextSample(iRef);
        count++;
    }
    return count;
}

uint8_t RxIoSampleResponse::getTemperature()
{
    return getFrameData()[4 + getExtendedFieldsLength()];
}

uint8_t RxIoSampleResponse::getVbatt()
{
    return getFrameData()[5 + getExtendedFieldsLength()];
}

uint8_t RxIoSampleResponse::getMode(uint8_t number)
{
    return getData()[getSample(number)];
}

uint8_t RxIoSampleResponse::getPin(uint8_t number)
{
    return getData()[getSample(number) - 1];
}

bool RxIoSampleResponse::isAvailable(uint8_t pin)
{
    for(uint8_t i = 1; i <= getSampleSize(); i++)
    {
        if(getPin(i) == pin)
          return true;
    }
    return false;
}

uint8_t RxIoSampleResponse::getSampleNumber(uint8_t pin)
{
    for(uint8_t i = 1; i <= getSampleSize(); i++)
    {
        if(getPin(i) == pin)
          return i;
    }
    return 0;
}

uint16_t RxIoSampleResponse::getAnalog(uint8_t number)
{
    uint16_t analog = (getData()[getSample(number) + 1] << 8) + getData()[getSample(number) + 2];
    if(analog > 0x7FFF)
        analog = 0; //Отрицательные числа(старший бит установлен) получаются в результате ошибок округления при коррекции и принимаются равными 0.
    return analog;
}

uint8_t RxIoSampleResponse::getDigital(uint8_t number)
{
    if(getData()[getSample(number)] & 0x80) //Информация о текущем состоянии цифрового входа/выхода передается в старшем бите кода режима.
        return HIGH;
    else
        return LOW;
}

uint32_t RxIoSampleResponse::getCounter(uint8_t number)
{
	return ((uint32_t)(getData()[getSample(number) + 1]) << 24) + ((uint32_t)(getData()[getSample(number) + 2]) << 16) + ((uint32_t)(getData()[getSample(number) + 3]) << 8) + getData()[getSample(number) + 4];
}

/*********************************************************************
* Private методы класса RxIoSampleResponse.
*********************************************************************/
void RxIoSampleResponse::getNextSample(uint8_t &alias)
{
    switch(getData()[alias] & 0x7F) //Сбрасываем старший бит, в котором передается информация о текущем состоянии цифровой линии.
    {
    case IO_ADC:
        alias += 4;
        break;
    case IO_DIGITAL_INPUT:
    case IO_DIGITAL_OUTPUT_LO:
    case IO_DIGITAL_OUTPUT_HI:
    case IO_WAKEUP_INPUT_FALLING_EDGE:
    case IO_WAKEUP_INPUT_RISING_EDGE:
        alias += 2;
        break;
    case IO_COUNTER_INPUT1:
    case IO_COUNTER_INPUT2:
        alias += 6;
        break;
    }
}

uint8_t RxIoSampleResponse::getSample(uint8_t number)
{
    uint8_t count = 1;
    uint8_t i = 1;
    uint8_t & iRef = i;
    while(i < getDataLength())
    {
        if(count == number)
            break;
        getNextSample(iRef);
        count++;
    }
    return i;
}

/*********************************************************************
* Public методы класса RxResponse.
*********************************************************************/
RxResponse::RxResponse() : RxDataResponse()
{

}

uint8_t RxResponse::getDataLength()
{
    return getFrameDataLength() - 4 - getExtendedFieldsLength();
}

uint8_t RxResponse::getDataOffset()
{
    return 4 + getExtendedFieldsLength();
}

bool RxResponse::isAcknowledged()
{
    return(getOption() & BV(PACKET_ACKNOWLEDGED_BIT));
}

/*********************************************************************
* Public методы класса MBeeRequest.
*********************************************************************/
MBeeRequest::MBeeRequest(uint8_t apiId, uint8_t frameId)
{
    _apiId = apiId;
    _frameId = frameId;
}

void MBeeRequest::setFrameId(uint8_t frameId)
{
    _frameId = frameId;
}

uint8_t MBeeRequest::getFrameId()
{
    return _frameId;
}

uint8_t MBeeRequest::getApiId()
{
    return _apiId;
}

void MBeeRequest::setApiId(uint8_t apiId)
{
    _apiId = apiId;
}

/*********************************************************************
* Public методы класса AtCommandRequest.
*********************************************************************/
AtCommandRequest::AtCommandRequest() : MBeeRequest(AT_COMMAND_API_FRAME, DEFAULT_FRAME_ID)
{
    _command = NULL;
    clearCommandValue();
}

AtCommandRequest::AtCommandRequest(uint8_t *command) : MBeeRequest(AT_COMMAND_API_FRAME, DEFAULT_FRAME_ID)
{
    _command = command;
    clearCommandValue();
}

AtCommandRequest::AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : MBeeRequest(AT_COMMAND_API_FRAME, DEFAULT_FRAME_ID)
{
    _command = command;
    _commandValue = commandValue;
    _commandValueLength = commandValueLength;
}

AtCommandRequest::AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength, uint8_t frameId) : MBeeRequest(AT_COMMAND_API_FRAME, frameId)
{
    _command = command;
    _commandValue = commandValue;
    _commandValueLength = commandValueLength;
}

uint8_t* AtCommandRequest::getCommand()
{
    return _command;
}

uint8_t* AtCommandRequest::getCommandValue()
{
    return _commandValue;
}

uint8_t AtCommandRequest::getCommandValueLength()
{
    return _commandValueLength;
}

void AtCommandRequest::setCommand(uint8_t* command)
{
    _command = command;
}

void AtCommandRequest::setCommandValue(uint8_t* value)
{
    _commandValue = value;
}

void AtCommandRequest::setCommandValueLength(uint8_t length)
{
    _commandValueLength = length;
}

uint8_t AtCommandRequest::getFrameData(uint8_t pos)
{
    if(pos == 0)
    {
        return _command[0];
    }
    else if(pos == 1)
    {
        return _command[1];
    }
    else
    {
        return _commandValue[pos - AT_COMMAND_API_LENGTH];
    }
}

void AtCommandRequest::clearCommandValue()
{
    _commandValue = NULL;
    _commandValueLength = 0;
}

uint8_t AtCommandRequest::getFrameDataLength()
{
    return AT_COMMAND_API_LENGTH + _commandValueLength;
}

/*********************************************************************
* Public методы класса PayloadRequest.
*********************************************************************/
PayloadRequest::PayloadRequest(uint8_t apiId, uint8_t frameId, uint8_t *payload, uint8_t payloadLength) : MBeeRequest(apiId, frameId)
{
    _payloadPtr = payload;
    _payloadLength = payloadLength;
}

uint8_t* PayloadRequest::getPayload()
{
    return _payloadPtr;
}

void PayloadRequest::setPayload(uint8_t* payload)
{
    _payloadPtr = payload;
}

uint8_t PayloadRequest::getPayloadLength()
{
    return _payloadLength;
}

void PayloadRequest::setPayloadLength(uint8_t payloadLength)
{
    _payloadLength = payloadLength;
}

/*********************************************************************
* Public методы класса RemoteAtCommandRequest.
*********************************************************************/
RemoteAtCommandRequest::RemoteAtCommandRequest() : AtCommandRequest(NULL, NULL, 0)
{
    _remoteAddress = BROADCAST_ADDRESS;
    _acknowledge = true;
    _applyChanges = true;
    _saveChanges = true;
    _cca = true;
    _encryption = false;
    _sleepingDevice = false;
    setApiId(REMOTE_AT_COMMAND_REQUEST_API_FRAME);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength)
{
    _remoteAddress = remoteAddress;
    _acknowledge = true;
    _applyChanges = true;
    _saveChanges = true;
    _cca = true;
    _encryption = false;
    _sleepingDevice = false;
    setApiId(REMOTE_AT_COMMAND_REQUEST_API_FRAME);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress, uint8_t *command) : AtCommandRequest(command, NULL, 0)
{
    _remoteAddress = remoteAddress;
    _acknowledge = true;
    _applyChanges = true;
    _saveChanges = true;
    _cca = true;
    _encryption = false;
    _sleepingDevice = false;
    setApiId(REMOTE_AT_COMMAND_REQUEST_API_FRAME);
}

uint16_t RemoteAtCommandRequest::getRemoteAddress()
{
    return _remoteAddress;
}

void RemoteAtCommandRequest::setRemoteAddress(uint16_t remoteAddress)
{
    _remoteAddress = remoteAddress;
}

bool RemoteAtCommandRequest::getAcknowledge()
{
    return _acknowledge;
}

void RemoteAtCommandRequest::setAcknowledge(bool acknowledge)
{
    _acknowledge = acknowledge;
}

bool RemoteAtCommandRequest::getApplyChanges()
{
    return _applyChanges;
}

void RemoteAtCommandRequest::setApplyChanges(bool applyChanges)
{
    _applyChanges = applyChanges;
}

bool RemoteAtCommandRequest::getSaveChanges()
{
    return _saveChanges;
}

void RemoteAtCommandRequest::setSaveChanges(bool saveChanges)
{
    _saveChanges = saveChanges;
}

bool RemoteAtCommandRequest::getCca()
{
    return _cca;
}

void RemoteAtCommandRequest::setCca(bool cca)
{
    _cca = cca;
}

bool RemoteAtCommandRequest::getEncryption()
{
    return _encryption;
}

void RemoteAtCommandRequest::setEncryption(bool encryption)
{
    _encryption = encryption;
}

bool RemoteAtCommandRequest::getSleepingDevice()
{
    return _sleepingDevice;
}

void RemoteAtCommandRequest::setSleepingDevice(bool sleepingDevice)
{
    _sleepingDevice = sleepingDevice;
}

uint8_t RemoteAtCommandRequest::getFrameData(uint8_t pos)
{
    if(pos == 0)
    {
        return (_remoteAddress >> 8) & 0xff;
    }
    else if(pos == 1)
    {
        return _remoteAddress & 0xff;
    }
    else if(pos == 2)
    {
        return ((_acknowledge ?  0: BV(ACKNOWLEDGE_DISABLE_COMMAND_OPTIONS_BIT)) +\
        (_applyChanges ? BV(APPLY_CHANGES_COMMAND_OPTION_BIT): 0) +\
        (_saveChanges ? 0: BV(APPLY_CHANGES_NO_SAVE_COMMAND_OPTION_BIT)) +\
        (_cca ?  0: BV(CCA_DISABLE_COMMAND_OPTION_BIT)) +\
        (_encryption ? BV(ENCRYPTION_ENABLE_COMMAND_OPTION_BIT): 0) +\
        (_sleepingDevice ? BV(SLEEP_DEVICE_COMMAND_OPTIONS_BIT): 0));
    }
    else if(pos == 3)
    {
        return getCommand()[0];
    }
    else if (pos == 4)
    {
        return getCommand()[1];
    }
    else
    {
        return getCommandValue()[pos - REMOTE_AT_COMMAND_API_LENGTH];
    }
}

uint8_t RemoteAtCommandRequest::getFrameDataLength()
{
    return REMOTE_AT_COMMAND_API_LENGTH + getCommandValueLength();
}

void RemoteAtCommandRequest::setDefault()
{
    _remoteAddress = BROADCAST_ADDRESS;
    _acknowledge = true;
    _applyChanges = true;
    _saveChanges = true;
    _cca = true;
    _encryption = false;
    _sleepingDevice = false;
    clearCommandValue();
}

/*********************************************************************
* Public методы класса TxRequest.
*********************************************************************/
TxRequest::TxRequest() : PayloadRequest(TRANSMIT_REQUEST_API_FRAME, DEFAULT_FRAME_ID, NULL, 0)
{
    _remoteAddress = BROADCAST_ADDRESS;
    _option = TRANSMIT_OPTION_DEFAULT;
    _optionEnable = true;
}

TxRequest::TxRequest(uint16_t remoteAddress, uint8_t *data, uint8_t dataLength) : PayloadRequest(TRANSMIT_REQUEST_API_FRAME, DEFAULT_FRAME_ID, data, dataLength)
{
    _remoteAddress = remoteAddress;
    _optionEnable = true;
    _option = TRANSMIT_OPTION_DEFAULT;
}

TxRequest::TxRequest(uint16_t remoteAddress, uint8_t option, uint8_t *data, uint8_t dataLength, uint8_t frameId) : PayloadRequest(TRANSMIT_REQUEST_API_FRAME, frameId, data, dataLength)
{
    _remoteAddress = remoteAddress;
    _optionEnable = true;
    _option = option;
}

uint16_t TxRequest::getRemoteAddress()
{
    return _remoteAddress;
}

void TxRequest::setRemoteAddress(uint16_t remoteAddress)
{
    _remoteAddress = remoteAddress;
}

uint8_t TxRequest::getOption()
{
    return _option;
}

void TxRequest::setOption(uint8_t option)
{
    _option = option;
}

bool TxRequest::getOptionEnable()
{
    return _optionEnable;
}

void TxRequest::setOptionEnable(bool optionEnable)
{
    _optionEnable = optionEnable;
    if(optionEnable)
        setApiId(TRANSMIT_REQUEST_API_FRAME);
    else
        setApiId(TRANSMIT_REQUEST_NO_OPTIONS_API_FRAME);
}

bool TxRequest::getAcknowledge()
{
    return (_option & BV(ACKNOWLEDGE_DISABLE_COMMAND_OPTIONS_BIT)) ? false : true;
}

void TxRequest::setAcknowledge(bool acknowledge)
{
    if(acknowledge)
        _option &= ~BV(ACKNOWLEDGE_DISABLE_COMMAND_OPTIONS_BIT);
    else
        _option |= BV(ACKNOWLEDGE_DISABLE_COMMAND_OPTIONS_BIT);
}

bool TxRequest::getCca()
{
    return (_option & BV(CCA_DISABLE_COMMAND_OPTION_BIT)) ? false : true;
}

void TxRequest::setCca(bool cca)
{
    if(cca)
        _option &= ~BV(CCA_DISABLE_COMMAND_OPTION_BIT);
    else
        _option |= BV(CCA_DISABLE_COMMAND_OPTION_BIT);
}

bool TxRequest::getEncryption()
{
    return (_option & BV(ENCRYPTION_ENABLE_COMMAND_OPTION_BIT)) ? true : false;
}

void TxRequest::setEncryption(bool encryption)
{
    if(encryption)
        _option |= BV(ENCRYPTION_ENABLE_COMMAND_OPTION_BIT);
    else
        _option &= ~BV(ENCRYPTION_ENABLE_COMMAND_OPTION_BIT);
}

bool TxRequest::getSleepingDevice()
{
    return (_option & BV(SLEEP_DEVICE_COMMAND_OPTIONS_BIT)) ? true : false;
}

void TxRequest::setSleepingDevice(bool sleepingDevice)
{
    if(sleepingDevice)
        _option |= BV(SLEEP_DEVICE_COMMAND_OPTIONS_BIT);
    else
        _option &= ~BV(SLEEP_DEVICE_COMMAND_OPTIONS_BIT);
}

uint8_t TxRequest::getFrameData(uint8_t pos)
{
    if(pos == 0)
    {
        return (_remoteAddress >> 8) & 0xFF;
    }
    else if(pos == 1)
    {
        return _remoteAddress & 0xFF;
    }
    else if((pos == 2) && _optionEnable)
    {
        return _option;
    }
    else
    {
        if(_optionEnable)
            return getPayload()[pos - 3];
        else
            return getPayload()[pos - 2];
    }
}

uint8_t TxRequest::getFrameDataLength()
{
    if(_optionEnable)
        return getPayloadLength() + 3;// Добавляется 2 байта адреса + 1 байт опций.
    else
        return getPayloadLength() + 2;// Добавляется только 2 байта адреса.
}

/*********************************************************************
* Public методы класса SerialStar.
*********************************************************************/
SerialStar::SerialStar(): _response(MBeeResponse())
{
    _pos = 0;
    _escape = false;
    _checksumTotal = 0;
    _nextFrameId = 0;

    _response.init();
    _response.setFrameData(_responseFrameData);
    //_serial = &Serial;
}

/*********************************************************************
* Переопределенные Public методы класса SerialStar, наследуемые из MBee868
*********************************************************************/
void SerialStar::readPacket()
{
    if(_response.isAvailable() || _response.isError())
        resetResponse(); //Сброс предыдущих сообщений.
    while(available())
    {
        b = read();
        if(_pos > 0 && b == START_BYTE && ATAP == 2)
        {
            //Обнаружен стартовый байт до окончания обработки предыдущего пакета. Сбрасываем предыдущее сообщение и начинаем разбор заново.
            _response.setErrorCode(UNEXPECTED_START_BYTE);
            return;
        }
        if(_pos > 0 && b == ESCAPE)
        {
            if(available())
            {
                b = read();
                b = 0x20 ^ b; //Извлекаем символ из escape-последовательности.
            }
            else
            {
                _escape = true; //Ставим флаг того, что следующий байт будет являться вторым символом из escape-последовательности.
                continue;
            }
        }
        if(_escape == true)
        {
            b = 0x20 ^ b;
            _escape = false; //Предыдущий байт был escape-символом. Извлекаем символ из escape-последовательности.
        }
        if(_pos >= API_ID_INDEX) //Контрольную сумму считаем начиная с байта типа API-фрейма.
            _checksumTotal+= b;

        //Конечный автомат парсера.
        switch(_pos)
        {
        case 0:
            if(b == START_BYTE)
                _pos++;
            break;
        case 1:
            _response.setMsbLength(b);
            _pos++;
            break;
        case 2:
            _response.setLsbLength(b);
            _pos++;
            break;
        case 3:
            _response.setApiId(b);
            _pos++;
            break;
        default:
            //Далее разбираются байты, следующие за 4-ым.
            if(_pos > MAX_FRAME_DATA_SIZE + 4)
            {
                _response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH); //Длина пакета превышает максимально допустимую.
                return;
            }
            //Проверяем, не достигли ли еще конца поля данных.
            if(_pos == (_response.getPacketLength() + 3))
            {
                if((_checksumTotal & 0xff) == 0xff)
                {
                    _response.setChecksum(b);
                    _response.setAvailable(true);
                    _response.setErrorCode(NO_ERROR_IN_FRAME);
                }
                else
                _response.setErrorCode(CHECKSUM_FAILURE); //Ошибка контрольной суммы.
                _response.setFrameLength(_pos - 4); //Длина поля данных не включает также поле Frame Type.
                _pos = 0;
                return;
            }
            else
            {
                _response.getFrameData()[_pos - 4] = b; //Добавляем считанный символ в поле данных.
                _pos++;
            }
        }
    }
}

bool SerialStar::readPacket(int timeout)
{
    if(timeout < 0)
    {
        return false;
    }
    unsigned long start = millis();
    while(int((millis() - start)) < timeout)
    {
        readPacket();
        if(getResponse().isAvailable())
        {
            return true;
        }
        else if(getResponse().isError())
        {
            return false;
        }
    }
    return false; //Таймаут..
}

void SerialStar::readPacketUntilAvailable()
{
    while(!(getResponse().isAvailable() || getResponse().isError()))
    {
        readPacket();
    }
}

void SerialStar::send(void* buffer)
{
    send((MBeeRequest&) buffer);
}

void SerialStar::send(MBeeRequest & request)
{
    sendByte(START_BYTE, false);
    uint8_t msbLen = ((request.getFrameDataLength() + 2) >> 8) & 0xFF;
    uint8_t lsbLen = (request.getFrameDataLength() + 2) & 0xFF;

    sendByte(msbLen, true);
    sendByte(lsbLen, true);

    sendByte(request.getApiId(), true);
    sendByte(request.getFrameId(), true);

    uint8_t checksum = 0;

    checksum+= request.getApiId();
    checksum+= request.getFrameId();

    for(int i = 0; i < request.getFrameDataLength(); i++)
    {
        sendByte(request.getFrameData(i), true);
        checksum+= request.getFrameData(i);
    }

    checksum = 0xff - checksum;
    sendByte(checksum, true);
}

MBeeResponse& SerialStar::getResponse()
{
    return _response;
}

/*********************************************************************
* Private методы класса SerialStar.
*********************************************************************/
void SerialStar::sendByte(uint8_t b, bool escape)
{
    // if(escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF))
    // {
    // write(ESCAPE);
    // write(b ^ 0x20);
    // }
    // else
    // {
    // write(b);
    // }
    write(b); //Проект SerialStar не поддерживает пакетный режим с escape-символами в направлении хост->модуль.
}
void SerialStar::resetResponse()
{
    _pos = 0;
    _escape = false;
    _checksumTotal = 0;
    _response.reset();
}

/*********************************************************************
* Public методы класса SerialStarWithCallbacks.
*********************************************************************/
void SerialStarWithCallbacks::run()
{
    if(loopTop())
        loopBottom();
}

uint8_t SerialStarWithCallbacks::sendAndWaitForAcknowledge(MBeeRequest &request, uint16_t timeout)
{
    uint8_t status;
    unsigned long start = millis();
    send(request);
    status = waitForStatus(request.getFrameId(), timeout - (millis() - start));
    if(status)
        return status;
    return waitForAcknowledge(request.getFrameId(), timeout - (millis() - start));
}

uint8_t SerialStarWithCallbacks::waitForStatus(uint8_t frameId, uint16_t timeout)
{
    unsigned long start = millis();
    do
    {
        if(loopTop())
        {
            uint8_t status = matchStatus(frameId);
            if(status != 0xff)
                return status;
            loopBottom(); //Вызываем регулярные callback-функции.
        }
    } while (millis() - start < timeout);
    return MBEE_WAIT_TIMEOUT ;
}

uint8_t SerialStarWithCallbacks::waitForAcknowledge(uint8_t frameId, uint16_t timeout)
{
    unsigned long start = millis();
    do
    {
        if(loopTop())
        {
            uint8_t status = matchAcknowledge(frameId);
            if(!status)
                return 0;
            loopBottom(); //Вызываем регулярные callback-функции.
        }
    } while (millis() - start < timeout);
    return MBEE_WAIT_TIMEOUT ;
}

/*********************************************************************
* Private методы класса SerialStarWithCallbacks.
*********************************************************************/
uint8_t SerialStarWithCallbacks::waitForInternal(uint8_t apiId, void *response, uint16_t timeout, void *func, uintptr_t data, int16_t frameId)
{
    unsigned long start = millis();
    do
    {
        if(loopTop()) //Ждем API-фрейм заданного типа.
        {
            if(frameId >= 0)
            {
                uint8_t status = matchStatus(frameId);
                if(status != 0xff && status != 0) //Если был принят фрейм со статусом и этот статус говорит о произошедшей ошибке, то прекращаем ожидание.
                return status;
            }
            if(getResponse().getApiId() == apiId)
            {
                //Если принят API-фрейм требуемого типа, то вызываем соответствующую apiId функцию преобразования типа принятого фрейма и затем функцию обработки.
                //Так как функции обработки вызываются одним и тем же способом, независимо от обрабатываемого подкласса (типа API-фрейма), то компилятор может
                //сократить всю эту кучу кода, находящуюся ниже в единственный блок. Хотя, для полной оптимизации, отдельные методы getXxxResponse() должны быть
                //унифицированы подобным образом.
                switch(apiId)
                {
                case MODEM_STATUS_API_FRAME:
                    {
                        ModemStatusResponse *r = (ModemStatusResponse*)response;
                        bool(*f)(ModemStatusResponse&,uintptr_t) = (bool(*)(ModemStatusResponse&,uintptr_t))func;
                        getResponse().getModemStatusResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case TRANSMIT_STATUS_API_FRAME:
                    {
                        TxStatusResponse *r = (TxStatusResponse*)response;
                        bool(*f)(TxStatusResponse&,uintptr_t) = (bool(*)(TxStatusResponse&,uintptr_t))func;
                        getResponse().getTxStatusResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case AT_COMMAND_RESPONSE_IMMEDIATE_APPLY_API_FRAME:
                case AT_COMMAND_RESPONSE_API_FRAME:
                case AT_COMMAND_RESPONSE_QUEUE_PARAMETER_VALUE_API_FRAME:
                    {
                        AtCommandResponse *r = (AtCommandResponse*)response;
                        bool(*f)(AtCommandResponse&,uintptr_t) = (bool(*)(AtCommandResponse&,uintptr_t))func;
                        getResponse().getAtCommandResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case REMOTE_AT_COMMAND_RESPONSE_API_FRAME:
                case REMOTE_AT_COMMAND_RESPONSE_EXTENDED_API_FRAME:
                    {
                        RemoteAtCommandResponse *r = (RemoteAtCommandResponse*)response;
                        bool(*f)(RemoteAtCommandResponse&,uintptr_t) = (bool(*)(RemoteAtCommandResponse&,uintptr_t))func;
                        getResponse().getRemoteAtCommandResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case REMOTE_ACKNOWLEDGE_API_FRAME:
                    {
                        RxAcknowledgeResponse *r = (RxAcknowledgeResponse*)response;
                        bool(*f)(RxAcknowledgeResponse&,uintptr_t) = (bool(*)(RxAcknowledgeResponse&,uintptr_t))func;
                        getResponse().getRxAcknowledgeResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case RECEIVE_PACKET_API_FRAME:
                case RECEIVE_PACKET_EXTENDED_API_FRAME:
                case RECEIVE_PACKET_NO_OPTIONS_API_FRAME:
                case RECEIVE_PACKET_NO_OPTIONS_EXTENDED_API_FRAME:
                    {
                        RxResponse *r = (RxResponse*)response;
                        bool(*f)(RxResponse&,uintptr_t) = (bool(*)(RxResponse&,uintptr_t))func;
                        getResponse().getRxResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }

                case IO_DATA_SAMPLE_API_FRAME:
                case IO_DATA_SAMPLE_EXTENDED_API_FRAME:
                    {
                        RxResponse *r = (RxResponse*)response;
                        bool(*f)(RxResponse&,uintptr_t) = (bool(*)(RxResponse&,uintptr_t))func;
                        getResponse().getRxResponse(*r);
                        if(!f || f(*r, data))
                            return 0;
                        break;
                    }
                }
            }
            loopBottom();
        }
    } while (millis() - start < timeout);
    return MBEE_WAIT_TIMEOUT;
}

uint8_t SerialStarWithCallbacks::matchStatus(uint8_t frameId)
{
    uint8_t id = getResponse().getApiId();
    uint8_t *data = getResponse().getFrameData();
    uint8_t len = getResponse().getFrameDataLength();
    uint8_t offset = 0;

    //Если фрейм имеет поле frameId, то определяем с каким смещением расположен в нем байт статуса.
    if((id == AT_COMMAND_RESPONSE_IMMEDIATE_APPLY_API_FRAME) || (id == AT_COMMAND_RESPONSE_API_FRAME) || (id == AT_COMMAND_RESPONSE_QUEUE_PARAMETER_VALUE_API_FRAME))
        offset = 3;
    else if(id == REMOTE_AT_COMMAND_RESPONSE_API_FRAME)
        offset = 6;
    else if(id == TRANSMIT_STATUS_API_FRAME)
        offset = 1;

    //Если фрейм содержит байт статуса (определяется по смещению не равному 0), если длина принятого фрейма больше, чем смещение (фрейм корректен), а также если
    //требуемый frameId совпадает с frameId, содержащимся в принятом фрейме, то считываем байт статуса.
    if(offset && offset < len && data[0] == frameId)
        return data[offset];
    return 0xff;
}

uint8_t SerialStarWithCallbacks::matchAcknowledge(uint8_t frameId)
{
    uint8_t id = getResponse().getApiId();
    uint8_t *data = getResponse().getFrameData();
    if(id == REMOTE_ACKNOWLEDGE_API_FRAME)
    {
        if(data[4] == frameId) //В 4-б байте передается frameId подтверждаемого пакета.
            return 0;
    }
    return 0xff;
}

bool SerialStarWithCallbacks::loopTop()
{
    readPacket();
    if(getResponse().isAvailable())
    {
        _onResponse.call(getResponse());
        return true;
    }
    else if(getResponse().isError())
    {
        _onPacketError.call(getResponse().getErrorCode());
    }
    return false;
}

void SerialStarWithCallbacks::loopBottom()
{
    bool called = false;
    uint8_t id = getResponse().getApiId();
    if(id == MODEM_STATUS_API_FRAME)
    {
        ModemStatusResponse response;
        getResponse().getModemStatusResponse(response);
        called = _onModemStatusResponse.call(response);
    }
    else if(id == TRANSMIT_STATUS_API_FRAME)
    {
        TxStatusResponse response;
        getResponse().getTxStatusResponse(response);
        called = _onTxStatusResponse.call(response);
    }
    else if((id == AT_COMMAND_RESPONSE_IMMEDIATE_APPLY_API_FRAME) || (id == AT_COMMAND_RESPONSE_API_FRAME) || (id == AT_COMMAND_RESPONSE_QUEUE_PARAMETER_VALUE_API_FRAME))
    {
        AtCommandResponse response;
        getResponse().getAtCommandResponse(response);
        called = _onAtCommandResponse.call(response);
    }
    else if((id == REMOTE_AT_COMMAND_RESPONSE_API_FRAME) || (id == REMOTE_AT_COMMAND_RESPONSE_EXTENDED_API_FRAME))
    {
        RemoteAtCommandResponse response;
        getResponse().getRemoteAtCommandResponse(response);
        called = _onRemoteAtCommandResponse.call(response);
    }
    else if(id == REMOTE_ACKNOWLEDGE_API_FRAME)
    {
        RxAcknowledgeResponse response;
        getResponse().getRxAcknowledgeResponse(response);
        called = _onRxAcknowledgeResponse.call(response);
    }
    else if((id == RECEIVE_PACKET_API_FRAME) || (id == RECEIVE_PACKET_NO_OPTIONS_API_FRAME) || (id == RECEIVE_PACKET_EXTENDED_API_FRAME) || (id == RECEIVE_PACKET_NO_OPTIONS_EXTENDED_API_FRAME))
    {
        RxResponse response;
        getResponse().getRxResponse(response);
        called = _onRxResponse.call(response);
    }
    else if((id == IO_DATA_SAMPLE_API_FRAME) || (id == IO_DATA_SAMPLE_EXTENDED_API_FRAME))
    {
        RxIoSampleResponse response;
        getResponse().getRxIoSampleResponse(response);
        called = _onRxIoSampleResponse.call(response);
    }

    if (!called)
    _onOtherResponse.call(getResponse());
}





