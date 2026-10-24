// sensordata.hpp
#pragma once

#include <QByteArray>
#include <QtEndian>
#include <optional>

// Sensor streams (head tracking, heart rate) use AACP opcode 0x17:
//   04 00 04 00 17 00 [descriptor, u32 LE] [length, u16 LE] [protobuf SensorDataWX]
// Field numbers follow android/core/src/main/proto/rtbuddy.proto on the android/rewrite branch.
// Only the fields needed here are encoded/decoded, with a minimal handwritten protobuf codec.
namespace AirPodsPackets
{
    namespace SensorData
    {
        static const QByteArray HEADER = QByteArray::fromHex("040004001700");
        constexpr quint32 SENSOR_DATA_WX_DESCRIPTOR = 0x00100000;

        enum class SensorService : quint32
        {
            Activity = 14,
            HeartRate = 19,
            HeartRateV2 = 20,
            HeartRateCommand = 84,
        };

        struct SensorCommand
        {
            quint32 service = 0;
            QByteArray payload;
        };

        namespace Protobuf
        {
            enum WireType : quint8
            {
                Varint = 0,
                Fixed64 = 1,
                LengthDelimited = 2,
                Fixed32 = 5,
            };

            struct Field
            {
                quint32 number = 0;
                WireType type = Varint;
                quint64 varint = 0; // Set for Varint fields
                QByteArray bytes;   // Set for LengthDelimited fields
            };

            inline void appendVarint(QByteArray &out, quint64 value)
            {
                while (value >= 0x80)
                {
                    out.append(static_cast<char>((value & 0x7F) | 0x80));
                    value >>= 7;
                }
                out.append(static_cast<char>(value));
            }

            inline void appendTag(QByteArray &out, quint32 field, WireType type)
            {
                appendVarint(out, (static_cast<quint64>(field) << 3) | type);
            }

            // proto3 omits fields holding the default value
            inline void appendVarintField(QByteArray &out, quint32 field, quint64 value)
            {
                if (value == 0)
                    return;
                appendTag(out, field, Varint);
                appendVarint(out, value);
            }

            inline void appendBytesField(QByteArray &out, quint32 field, const QByteArray &value)
            {
                appendTag(out, field, LengthDelimited);
                appendVarint(out, static_cast<quint64>(value.size()));
                out.append(value);
            }

            inline std::optional<quint64> readVarint(const QByteArray &data, int &pos)
            {
                quint64 value = 0;
                for (int shift = 0; shift < 64 && pos < data.size(); shift += 7)
                {
                    quint8 byte = static_cast<quint8>(data.at(pos++));
                    value |= static_cast<quint64>(byte & 0x7F) << shift;
                    if (!(byte & 0x80))
                        return value;
                }
                return std::nullopt;
            }

            // Calls handler(field) for each field in the message. Returns false on malformed input.
            template <typename Handler>
            bool forEachField(const QByteArray &data, Handler handler)
            {
                int pos = 0;
                while (pos < data.size())
                {
                    auto tag = readVarint(data, pos);
                    if (!tag)
                        return false;

                    Field field;
                    field.number = static_cast<quint32>(*tag >> 3);
                    field.type = static_cast<WireType>(*tag & 0x07);

                    switch (field.type)
                    {
                    case Varint:
                    {
                        auto value = readVarint(data, pos);
                        if (!value)
                            return false;
                        field.varint = *value;
                        break;
                    }
                    case LengthDelimited:
                    {
                        auto length = readVarint(data, pos);
                        if (!length || *length > static_cast<quint64>(data.size() - pos))
                            return false;
                        field.bytes = data.mid(pos, static_cast<int>(*length));
                        pos += static_cast<int>(*length);
                        break;
                    }
                    case Fixed64:
                        if (data.size() - pos < 8)
                            return false;
                        pos += 8;
                        break;
                    case Fixed32:
                        if (data.size() - pos < 4)
                            return false;
                        pos += 4;
                        break;
                    default:
                        return false; // Groups are not used
                    }

                    handler(field);
                }
                return true;
            }
        }

        // SensorDataWX { seq = 1; service_settings = 8 { service = 1; setting = 2; configuration = 3 } }
        // Setting 2 with configuration [0x01, interval in microseconds as u32 LE] sets how often the
        // service reports. An interval of 0 stops the stream.
        inline QByteArray createReportIntervalPacket(quint32 sequence, SensorService service, quint32 intervalUs)
        {
            QByteArray configuration(1, 0x01);
            char interval[4];
            qToLittleEndian(intervalUs, interval);
            configuration.append(interval, sizeof(interval));

            QByteArray settings;
            Protobuf::appendVarintField(settings, 1, static_cast<quint32>(service));
            Protobuf::appendVarintField(settings, 2, 2);
            Protobuf::appendBytesField(settings, 3, configuration);

            QByteArray message;
            Protobuf::appendVarintField(message, 1, sequence);
            Protobuf::appendBytesField(message, 8, settings);

            char descriptor[4];
            qToLittleEndian(SENSOR_DATA_WX_DESCRIPTOR, descriptor);
            char length[2];
            qToLittleEndian(static_cast<quint16>(message.size()), length);

            QByteArray packet = HEADER;
            packet.append(descriptor, sizeof(descriptor));
            packet.append(length, sizeof(length));
            packet.append(message);
            return packet;
        }

        // Returns the SensorDataWX protobuf bytes of a sensor data packet
        inline std::optional<QByteArray> extractMessage(const QByteArray &data)
        {
            const int headerSize = HEADER.size() + 6; // Descriptor and length
            if (!data.startsWith(HEADER) || data.size() < headerSize)
                return std::nullopt;

            quint32 descriptor = qFromLittleEndian<quint32>(data.constData() + HEADER.size());
            quint16 length = qFromLittleEndian<quint16>(data.constData() + HEADER.size() + 4);
            if (descriptor != SENSOR_DATA_WX_DESCRIPTOR || data.size() < headerSize + length)
                return std::nullopt;

            return data.mid(headerSize, length);
        }

        // The AirPods send samples in the command field: SensorDataWX { command = 7 { service = 1; payload = 3 } }
        inline std::optional<SensorCommand> parseCommand(const QByteArray &data)
        {
            auto message = extractMessage(data);
            if (!message)
                return std::nullopt;

            std::optional<SensorCommand> command;
            bool valid = Protobuf::forEachField(*message, [&command](const Protobuf::Field &field)
            {
                if (field.number != 7 || field.type != Protobuf::LengthDelimited)
                    return;

                SensorCommand parsed;
                bool commandValid = Protobuf::forEachField(field.bytes, [&parsed](const Protobuf::Field &inner)
                {
                    if (inner.number == 1 && inner.type == Protobuf::Varint)
                        parsed.service = static_cast<quint32>(inner.varint);
                    else if (inner.number == 3 && inner.type == Protobuf::LengthDelimited)
                        parsed.payload = inner.bytes;
                });
                if (commandValid)
                    command = parsed;
            });

            return valid ? command : std::nullopt;
        }

        // Heart rate payloads are 18 bytes. Byte 1 is the BPM (unsigned) and samples with a non-zero
        // byte 17 are dropped (meaning unknown, probably no reliable reading yet). The other bytes are
        // not understood. Based on android/rewrite; not yet tested with real AirPods on Linux.
        inline std::optional<int> parseHeartRate(const SensorCommand &command)
        {
            if (command.service != static_cast<quint32>(SensorService::HeartRate) &&
                command.service != static_cast<quint32>(SensorService::HeartRateV2))
                return std::nullopt;

            if (command.payload.size() != 18 || command.payload.at(17) != 0)
                return std::nullopt;

            return static_cast<quint8>(command.payload.at(1));
        }
    }
}
