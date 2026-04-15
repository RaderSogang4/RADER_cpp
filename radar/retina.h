#pragma once

#include <ostream>
#include <vector>
#include <array>
#include <mutex>

#define ASIO_STANDALONE
#include <asio.hpp>

namespace retina
{
	using asio::ip::tcp;

	enum TargetStatus : uint32_t
	{
		Standing = 0,
		Sitting = 1,
		Lying = 2,
		Walking = 4,
	};

	struct Point
	{
		float x;
		float y;
		float z;
		float doppler;
		float power;
		int32_t targetId;
	};
	
	struct Target
	{
		float x;
		float y;
		TargetStatus status;
		uint32_t targetId;
		float minx;
		float maxx;
		float miny;
		float maxy;
		float minz;
		float maxz;
	};
	
	struct Frame
	{
		uint32_t packetSize;
		uint32_t frameCount;
		std::vector<Point> points;
		std::vector<Target> targets;
	};

	static const char* to_string(retina::TargetStatus status)
	{
		switch (status)
		{
			case retina::TargetStatus::Standing: return "Standing";
			case retina::TargetStatus::Sitting: return "Sitting";
			case retina::TargetStatus::Lying: return "Lying";
			case retina::TargetStatus::Walking: return "Walking";
			default: return "Unknown";
		}
	}

	class DeviceFinder
	{
	public:
		DeviceFinder(asio::io_context& io, std::ostream& log_stream);

		bool find(const std::string& local_ip, const std::string& subnet_mask, std::string& out_host);

	private:
		bool buildScanRange(const std::string& local_ip, const std::string& subnet_mask, uint32_t& out_network, uint32_t& out_first, uint32_t& out_last);
		bool probeHost(const std::string& ip);
		bool probeTcpPort(const std::string& ip, uint16_t port, std::chrono::milliseconds timeout);
		bool probeHttpPort(const std::string& ip, std::chrono::milliseconds timeout);
		std::string formatAddress(uint32_t address) const;

	private:
		asio::io_context& m_io;
		std::ostream& m_log;
	};

	class DeviceClient
	{
	public:
		static constexpr uint32_t frame_count_limit_default = 50;

		DeviceClient(asio::io_context& io, std::string host, std::ostream& log_stream);

		void setFrameCountLimit(uint32_t limit);

		void readFrames(std::function<void(const std::vector<Frame>&)> callback);
	
	private:
		void doRead();
		bool tryExtractPacket(std::vector<uint8_t>& stream_buf, std::vector<uint8_t>& out_packet_buf);
		bool parseSinglePacket(const std::vector<uint8_t>& packet_buf, Frame& out_frame);
		bool findPacketMagic(const std::vector<uint8_t>& buffer, size_t& out_offset);

	private:
		asio::io_context& m_io;
		std::ostream& m_log;

		tcp::resolver m_resolver;
		tcp::socket m_socket;
		uint32_t m_frame_count_limit;

		mutable std::vector<Frame> m_frames;

		std::array<uint8_t, 4096> m_read_buf {};
		std::vector<uint8_t> m_stream_buf;
		std::vector<uint8_t> m_packet_buf;
		std::mutex m_mutex;
	};
}