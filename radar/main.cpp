#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot3d.h>
#include <GLFW/glfw3.h>

#include "network.h"
#include "retina.h"
#include "debug_console.h"
#include "implot3d_custom.h"

static float safe_span(float min_value, float max_value)
{
	const float span = max_value - min_value;
	return span > 1.0e-6f ? span : 1.0f;
}

class Exception : public std::exception
{
public:
	Exception(const std::string& message) :
		m_message(message)
	{
	}
	
	const char* what() const noexcept override
	{
		return m_message.c_str();
	}

private:
	std::string m_message;
};

struct RadarFrameRateTracker
{
	bool HasFrame = false;
	uint32_t LastFrameCount = 0;
	std::chrono::steady_clock::time_point LastFrameTime {};
	float Fps = 0.0f;

	void OnFrame(uint32_t frame_count)
	{
		const auto now = std::chrono::steady_clock::now();
		if (!HasFrame)
		{
			HasFrame = true;
			LastFrameCount = frame_count;
			LastFrameTime = now;
			return;
		}
		if (frame_count == LastFrameCount)
			return;

		const auto elapsed = std::chrono::duration<float>(now - LastFrameTime).count();
		if (elapsed > 0.0f)
		{
			const uint32_t frame_delta = frame_count > LastFrameCount ? (frame_count - LastFrameCount) : 1u;
			const float instant_fps = (float)frame_delta / elapsed;
			Fps = Fps > 0.0f ? (Fps * 0.8f + instant_fps * 0.2f) : instant_fps;
		}

		LastFrameCount = frame_count;
		LastFrameTime = now;
	}
};

class MyApp
{
public:
	static constexpr ImPlot3DQuat initial_rotation = ImPlot3DQuat(-0.513269, -0.212596, -0.318184, 0.76819);

	static constexpr float x_range_min = -2.0f;
	static constexpr float x_range_max = 2.0f;
	static constexpr float y_range_min = 0.0f;
	static constexpr float y_range_max = 9.0f;
	static constexpr float z_range_min = -2.0f;
	static constexpr float z_range_max = 2.0f;
	static constexpr ImPlot3DBox axis_box{
		{ x_range_min, y_range_min, z_range_min },
		{ x_range_max, y_range_max, z_range_max }
	};

	MyApp() :
		m_io(),
		m_console(),
		m_log(m_console.getOutput())
	{
		initWindow();
		beginAsyncIO();
	}

	~MyApp()
	{
		m_io.stop();
	
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot3D::DestroyContext();
		ImGui::DestroyContext();
		glfwDestroyWindow(m_window);
		glfwTerminate();
		
		if (m_io_thread.joinable())
			m_io_thread.join();
	}

	void run()
	{
		retina::Frame frame{};
		RadarFrameRateTracker tracker;

		while (!glfwWindowShouldClose(m_window))
		{
			glfwPollEvents();
			if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED) != 0)
			{
				glfwWaitEventsTimeout(0.01);
				continue;
			}

			if (m_client)
			{
				m_client->readFrames([&](const std::vector<retina::Frame>& frames)
					{
						if (!frames.empty())
						{
							frame = frames.back();
							tracker.OnFrame(frame.frameCount);
						}
					});
			}

			renderUI(frame, tracker);
		}
	}

private:
	void initWindow()
	{
		glfwSetErrorCallback(glfw_error_callback);
		
		if (!glfwInit())
			throw Exception("[glfw] failed to initialize GLFW");

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

		m_window = glfwCreateWindow(1280, 800, "Radar Visualization", nullptr, nullptr);

		if (m_window == nullptr)
			throw std::exception("[glfw] failed to create window");

		glfwMakeContextCurrent(m_window);
		glfwSwapInterval(1);

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImPlot3D::CreateContext();

		if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
			throw Exception("[imgui] failed to initialize GLFW backend");
		if (!ImGui_ImplOpenGL3_Init("#version 130"))
			throw Exception("[imgui] failed to initialize OpenGL3 backend");

		ImGuiIO& imgui_io = ImGui::GetIO();
		imgui_io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
		m_console.setFont(imgui_io.Fonts->AddFontDefault());

		if (ImFont* font = imgui_io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\consola.ttf", 18.0f))
			imgui_io.FontDefault = font;

		applyTheme();
	}

	void applyTheme()
	{
		ImGui::StyleColorsDark();
		ImGuiStyle& style = ImGui::GetStyle();
		style.WindowPadding = ImVec2(8.0f, 8.0f);
		style.FramePadding = ImVec2(6.0f, 4.0f);
		style.ItemSpacing = ImVec2(6.0f, 5.0f);
		style.ScrollbarSize = 14.0f;
		style.GrabMinSize = 12.0f;

		ImVec4* colors = style.Colors;
		colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.10f, 1.00f);
		colors[ImGuiCol_ChildBg] = ImVec4(0.06f, 0.07f, 0.08f, 1.00f);
		colors[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.09f, 0.10f, 0.98f);
		colors[ImGuiCol_Border] = ImVec4(0.20f, 0.22f, 0.25f, 0.80f);
		colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.14f, 0.16f, 1.00f);
		colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.20f, 0.23f, 1.00f);
		colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.24f, 0.28f, 1.00f);
		colors[ImGuiCol_TitleBg] = ImVec4(0.06f, 0.07f, 0.08f, 1.00f);
		colors[ImGuiCol_TitleBgActive] = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
		colors[ImGuiCol_MenuBarBg] = ImVec4(0.07f, 0.08f, 0.09f, 1.00f);
		colors[ImGuiCol_ScrollbarBg] = ImVec4(0.04f, 0.05f, 0.06f, 0.80f);
		colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.25f, 0.27f, 0.30f, 1.00f);
		colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.33f, 0.36f, 0.40f, 1.00f);
		colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.38f, 0.42f, 0.46f, 1.00f);
		colors[ImGuiCol_Button] = ImVec4(0.15f, 0.17f, 0.20f, 1.00f);
		colors[ImGuiCol_ButtonHovered] = ImVec4(0.20f, 0.23f, 0.27f, 1.00f);
		colors[ImGuiCol_ButtonActive] = ImVec4(0.24f, 0.27f, 0.31f, 1.00f);
		colors[ImGuiCol_Header] = ImVec4(0.16f, 0.18f, 0.21f, 1.00f);
		colors[ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.25f, 0.29f, 1.00f);
		colors[ImGuiCol_HeaderActive] = ImVec4(0.24f, 0.28f, 0.32f, 1.00f);
		colors[ImGuiCol_Separator] = ImVec4(0.20f, 0.22f, 0.25f, 1.00f);
		colors[ImGuiCol_SeparatorHovered] = ImVec4(0.28f, 0.32f, 0.36f, 1.00f);
		colors[ImGuiCol_SeparatorActive] = ImVec4(0.32f, 0.36f, 0.40f, 1.00f);

		ImPlot3D::StyleColorsDark();
		ImPlot3DStyle& plot_style = ImPlot3D::GetStyle();
		plot_style.PlotPadding = ImVec2(2.0f, 2.0f);
		plot_style.LabelPadding = ImVec2(3.0f, 3.0f);
		plot_style.LegendPadding = ImVec2(6.0f, 6.0f);
		plot_style.LegendInnerPadding = ImVec2(4.0f, 3.0f);
		plot_style.LegendSpacing = ImVec2(8.0f, 4.0f);
		plot_style.ViewScaleFactor = 1.0f;
		plot_style.Colors[ImPlot3DCol_PlotBg] = ImVec4(0.05f, 0.06f, 0.07f, 1.00f);
		plot_style.Colors[ImPlot3DCol_FrameBg] = ImVec4(0.12f, 0.13f, 0.15f, 1.00f);
		plot_style.Colors[ImPlot3DCol_LegendBg] = ImVec4(0.07f, 0.08f, 0.09f, 0.98f);
		plot_style.Colors[ImPlot3DCol_LegendBorder] = ImVec4(0.20f, 0.22f, 0.25f, 0.85f);
	}

	void beginAsyncIO()
	{
		m_io_thread = std::thread([this]()
			{
				network::LocalNetworkInfo network_info;
				if (!network::getLocalNetworkInfo(network_info))
				{
					m_log << "[error] [net] failed to determine local network information\n";
					return;
				}

				retina::DeviceFinder finder(m_io, m_log);
				std::string host;

				if (!finder.find(network_info.Address, network_info.SubnetMask, host))
				{
					m_log << "[error] [net] failed to discover retina device on the local network\n";
					return;
				}

				m_log << "[net] retina device found at " << host << '\n';

				m_client = std::make_unique<retina::DeviceClient>(m_io, host, m_log);

				try
				{
					m_io.restart();
					m_io.run();
				}
				catch (const std::exception& e)
				{
					m_log << "[fatal] exception: " << e.what() << "\n";
				}
			});
	}

	void renderUI(const retina::Frame& frame, const RadarFrameRateTracker& tracker)
	{
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGuiViewport* viewport = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(viewport->WorkPos);
		ImGui::SetNextWindowSize(viewport->WorkSize);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));

		ImGuiWindowFlags windowFlags =
			ImGuiWindowFlags_MenuBar |
			ImGuiWindowFlags_NoDecoration |
			ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_NoBringToFrontOnFocus |
			ImGuiWindowFlags_NoNavFocus;

		ImGui::Begin("Cloud View", nullptr, windowFlags);

		drawMenu();

		const ImVec2 content_available = ImGui::GetContentRegionAvail();
		const float top_height = content_available.y;
		const float min_sidebar_width = 260.0f;
		const float max_sidebar_width = content_available.x * 0.36f;
		const float splitter_width = 6.0f;
		m_sidebar_width = std::clamp(m_sidebar_width, min_sidebar_width, max_sidebar_width);
		const float plot_width = std::max(content_available.x - m_sidebar_width - splitter_width, 0.f);

		ImPlot3DFlags plot_flags =
			ImPlot3DFlags_NoZoom;

		if (ImPlot3D::BeginPlot("Point Cloud", ImVec2(plot_width, top_height), plot_flags))
		{
			ImPlot3DSpec spec;
			spec.Marker = ImPlot3DMarker_Circle;
			spec.MarkerSize = 2.0f;
			spec.FillAlpha = 1.0f;
			spec.Flags = ImPlot3DItemFlags_NoLegend;

			ImPlot3D::SetupAxis(ImAxis3D_X, "X");
			ImPlot3D::SetupAxis(ImAxis3D_Y, "Y");
			ImPlot3D::SetupAxis(ImAxis3D_Z, "Z");

			// Setup inital plot axis limits and scales
			ImPlot3D::SetupAxesLimits(
				x_range_min, x_range_max,
				y_range_min, y_range_max,
				z_range_min, z_range_max);
			ImPlot3D::SetupBoxScale(
				1.f,
				(y_range_max - y_range_min) / (x_range_max - x_range_min),
				(z_range_max - z_range_min) / (x_range_max - x_range_min));

			ImPlot3D::ShowViewGizmo("##MainViewGizmo", 104.0f, &m_view_rotation);

			ImPlot3D::PlotScatter(
				"##RadarScatter",
				this,
				frame.points.data(),
				static_cast<int>(frame.points.size()),
				sizeof(retina::Point),
				scatter_getter,
				spec);

			if (m_show_targets)
			{
				spec.Marker = ImPlot3DMarker_None;
				spec.FillAlpha = 0.1f;

				for (const auto& target : frame.targets)
				{
					auto text_x = (target.minx + target.maxx) * 0.5f;
					auto text_y = (target.miny + target.maxy) * 0.5f;
					auto text_z = target.maxz + 0.3f;

					std::string text = "ID: " + std::to_string(target.targetId) + '(' + retina::to_string(target.status) + ')';

					ImPlot3D::PlotText(text.c_str(), text_x, text_y, text_z);
					ImPlot3D::PlotBox("##RadarBox", target.minx, target.maxx, target.miny, target.maxy, target.minz, target.maxz, spec);
				}
			}

			if (m_show_frustum)
				drawRadarFrustum(axis_box);

			ImPlot3D::EndPlot();
		}

		ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);
		ImGui::InvisibleButton("##SidebarSplitter", ImVec2(splitter_width, top_height));
		if (ImGui::IsItemHovered() || ImGui::IsItemActive())
			ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
		if (ImGui::IsItemActive())
			m_sidebar_width = std::clamp(m_sidebar_width - ImGui::GetIO().MouseDelta.x, min_sidebar_width, max_sidebar_width);

		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		const ImVec2 splitter_min = ImGui::GetItemRectMin();
		const ImVec2 splitter_max = ImGui::GetItemRectMax();
		const ImU32 splitter_color = ImGui::GetColorU32(ImGui::IsItemActive() ? ImGuiCol_SeparatorActive : ImGui::IsItemHovered() ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator);
		draw_list->AddRectFilled(splitter_min, splitter_max, splitter_color, 2.0f);

		ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);
		drawSidebar(frame, tracker, { m_sidebar_width, top_height });

		ImGui::PopStyleVar(3);
		ImGui::End();

		if (m_show_xy)
			darwRadarProjectionWindow(
				"XY Projection",
				frame,
				axis_box,
				ImAxis3D_Z,
				&m_show_xy);
		
		if (m_show_xz)
			darwRadarProjectionWindow(
				"XZ Projection",
				frame,
				axis_box,
				ImAxis3D_Y,
				&m_show_xz);
		
		if (m_show_yz)
			darwRadarProjectionWindow(
				"YZ Projection",
				frame,
				axis_box,
				ImAxis3D_X,
				&m_show_yz);

		ImGui::Render();

		int displayWidth = 0;
		int displayHeight = 0;
		glfwGetFramebufferSize(m_window, &displayWidth, &displayHeight);
		glViewport(0, 0, displayWidth, displayHeight);
		glClearColor(0.1f, 0.1f, 0.1f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(m_window);
	}

	void drawMenu()
	{
		if (ImGui::BeginMenuBar())
		{
			if (ImGui::BeginMenu("View"))
			{
				ImGui::MenuItem("Show Frustum", nullptr, &m_show_frustum);
				ImGui::Separator();
				ImGui::MenuItem("Show Targets", nullptr, &m_show_targets);
				ImGui::Separator();

				if (ImGui::MenuItem("Reset View"))
					m_view_rotation = initial_rotation;

				ImGui::Separator();
				
				if (ImGui::MenuItem("Left"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_X, true);
				if (ImGui::MenuItem("Right"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_X, false);
				if (ImGui::MenuItem("Rear"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_Y, false);
				if (ImGui::MenuItem("Front"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_Y, true);
				if (ImGui::MenuItem("Bottom"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_Z, true);
				if (ImGui::MenuItem("Top"))
					m_view_rotation = getAxisAlignedViewRotation(ImAxis3D_Z, false);

				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Window"))
			{
				ImGui::MenuItem("XY Projection", nullptr, &m_show_xy);
				ImGui::MenuItem("XZ Projection", nullptr, &m_show_xz);
				ImGui::MenuItem("YZ Projection", nullptr, &m_show_yz);
				ImGui::EndMenu();
			}
			ImGui::EndMenuBar();
		}
	}

	void drawSidebar(const retina::Frame& frame, const RadarFrameRateTracker& frame_tracker, const ImVec2& size)
	{
		ImGui::BeginChild("##Sidebar", size, true);

		ImGui::Text("Frame  : %u", frame.frameCount);
		ImGui::Text("Update : %.1f FPS", frame_tracker.Fps);
		ImGui::Text("Packet : %u bytes", frame.packetSize);
		ImGui::Text("Points : %zu", frame.points.size());
		ImGui::Text("Targets: %zu", frame.targets.size());

		ImGui::Separator();

		ImGui::TextUnformatted("Doppler -> Color");
		drawLegendBar(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::DragFloatRange2("##Doppler Range", &m_doppler_min, &m_doppler_max, 0.01f, -10.0f, 10.0f, "%.2f", "%.2f");
		ImGui::Spacing();

		ImGui::TextUnformatted("Power -> Alpha");
		drawLegendBar(ImVec4(0.f, 0.f, 0.0f, 1.f), ImVec4(1.f, 1.f, 1.0f, 1.0f));
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::DragFloatRange2("##Power Range", &m_power_min, &m_power_max, 100.0f, 0.0f, 1000000.0f, "%.0f", "%.0f");
		
		if (ImGui::Button("Reset Ranges"))
		{
			m_doppler_min = 0.0f;
			m_doppler_max = 2.0f;
			m_power_min = 0.0f;
			m_power_max = 100000.0f;
		}

		ImGui::Separator();

		const float console_height = ImGui::GetContentRegionAvail().y;
		m_console.draw("console", ImVec2(-1.0f, console_height));

		ImGui::EndChild();
	}

	void drawRadarFrustum(const ImPlot3DBox& axis_box)
	{
		constexpr float hfov_deg = 90.f;
		constexpr float vfov_deg = 90.f;
		constexpr float sensor_width = 0.13f;
		constexpr float sensor_height = 0.13f;
		constexpr float pi = 3.14159265358979323846f;
		const float near_distance = std::max<float>(0.0, axis_box.Min.y);
		const float far_distance = std::max<float>(near_distance, axis_box.Max.y);
		const float tan_half_hfov = tan(hfov_deg * 0.5f * pi / 180.f);
		const float tan_half_vfov = tan(vfov_deg * 0.5f * pi / 180.f);
		const float sensor_half_w = sensor_width * 0.5f;
		const float sensor_half_h = sensor_height * 0.5f;
		const float focal_x = sensor_half_w / tan_half_hfov;
		const float focal_y = sensor_half_h / tan_half_vfov;

		if (far_distance < near_distance || focal_x <= 0.f || focal_y <= 0.f)
			return;

		auto make_plane_corners = [&](float plane_y)
		{
			const float depth_from_sensor = plane_y - near_distance;
			const float half_w = sensor_half_w + depth_from_sensor * tan_half_hfov;
			const float half_h = sensor_half_h + depth_from_sensor * tan_half_vfov;

			return std::array<ImPlot3DPoint, 4> {
				ImPlot3DPoint(-half_w, plane_y, -half_h),
				ImPlot3DPoint(half_w, plane_y, -half_h),
				ImPlot3DPoint(half_w, plane_y, half_h),
				ImPlot3DPoint(-half_w, plane_y, half_h)
			};
		};

		std::array<float, 24> xs;
		std::array<float, 24> ys;
		std::array<float, 24> zs;
		size_t idx = 0;

		auto add_segment = [&](double x0, double y0, double z0, double x1, double y1, double z1)
		{
			xs[idx] = static_cast<float>(x0); ys[idx] = static_cast<float>(y0); zs[idx] = static_cast<float>(z0); ++idx;
			xs[idx] = static_cast<float>(x1); ys[idx] = static_cast<float>(y1); zs[idx] = static_cast<float>(z1); ++idx;
		};

		const std::array<ImPlot3DPoint, 4> near_corners = make_plane_corners(near_distance);
		const std::array<ImPlot3DPoint, 4> far_corners = make_plane_corners(far_distance);

		for (int i = 0; i < 4; ++i)
		{
			const int next = (i + 1) % 4;
			add_segment(near_corners[i].x, near_corners[i].y, near_corners[i].z, near_corners[next].x, near_corners[next].y, near_corners[next].z);
			add_segment(far_corners[i].x, far_corners[i].y, far_corners[i].z, far_corners[next].x, far_corners[next].y, far_corners[next].z);
			add_segment(near_corners[i].x, near_corners[i].y, near_corners[i].z, far_corners[i].x, far_corners[i].y, far_corners[i].z);
		}

		ImPlot3DSpec spec;
		spec.LineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.85f);
		spec.LineWeight = 1.5f;
		spec.Marker = ImPlot3DMarker_None;
		spec.Flags = ImPlot3DItemFlags_NoLegend | ImPlot3DLineFlags_Segments;

		ImPlot3D::PlotLine("##RadarFrustum", xs.data(), ys.data(), zs.data(), (int)idx, spec);
	}

	void drawLegendBar(const ImVec4& left, const ImVec4& right)
	{
		ImVec2 size(ImGui::GetContentRegionAvail().x, 18.0f);
		size.x = size.x > 1.0f ? size.x : 1.0f;

		const ImVec2 p0 = ImGui::GetCursorScreenPos();
		const ImVec2 p1(p0.x + size.x, p0.y + size.y);
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		draw_list->AddRectFilled(p0, p1, IM_COL32(32, 32, 36, 255), 3.0f);
		draw_list->AddRectFilledMultiColor(
			p0,
			p1,
			ImGui::ColorConvertFloat4ToU32(left),
			ImGui::ColorConvertFloat4ToU32(right),
			ImGui::ColorConvertFloat4ToU32(right),
			ImGui::ColorConvertFloat4ToU32(left));
		draw_list->AddRect(p0, p1, IM_COL32(255, 255, 255, 48), 3.0f);
		ImGui::Dummy(size);
	}

	void darwRadarProjectionWindow(
		const char* title,
		const retina::Frame& frame,
		const ImPlot3DBox& axis_ranges,
		ImAxis3D normal_axis,
		bool* p_open)
	{
		ImGui::SetNextWindowSize(ImVec2(360.0f, 320.0f), ImGuiCond_FirstUseEver);
		if (!ImGui::Begin(title, p_open))
		{
			ImGui::End();
			return;
		}

		const ImPlot3DFlags flags =
			ImPlot3DFlags_NoLegend |
			ImPlot3DFlags_NoMouseText |
			ImPlot3DFlags_NoMenus;
		
		if (ImPlot3D::BeginPlot("##ProjectionPlot", ImVec2(-1.0f, -1.0f), flags))
		{
			ImPlot3DAxisFlags axis_flags[3] = {
				ImPlot3DAxisFlags_None,
				ImPlot3DAxisFlags_None,
				ImPlot3DAxisFlags_None,
			};
			axis_flags[normal_axis] = ImPlot3DAxisFlags_NoDecorations;

			ImPlot3D::SetupAxes("X", "Y", "Z", axis_flags[0], axis_flags[1], axis_flags[2]);
			ImPlot3D::SetupAxesLimits(axis_ranges.Min.x, axis_ranges.Max.x, axis_ranges.Min.y, axis_ranges.Max.y, axis_ranges.Min.z, axis_ranges.Max.z);
			ImPlot3D::SetupBoxScale(
				1.0,
				safe_span(axis_ranges.Min.y, axis_ranges.Max.y) / safe_span(axis_ranges.Min.x, axis_ranges.Max.x),
				safe_span(axis_ranges.Min.z, axis_ranges.Max.z) / safe_span(axis_ranges.Min.x, axis_ranges.Max.x));
			ImPlot3D::SetupBoxRotation(getAxisAlignedViewRotation(normal_axis, true), false, ImPlot3DCond_Always);

			ImPlot3DSpec spec;
			spec.Marker = ImPlot3DMarker_Circle;
			spec.MarkerSize = 2.0f;
			spec.FillAlpha = 1.0f;
			spec.Flags = ImPlot3DItemFlags_NoLegend;
			
			ImPlot3D::PlotScatter(
				"##ProjectionScatter",
				this,
				frame.points.data(),
				static_cast<int>(frame.points.size()),
				static_cast<int>(sizeof(retina::Point)),
				scatter_getter,
				spec);
			
			ImPlot3D::EndPlot();
		}

		ImGui::End();
	}

	ImPlot3DQuat getAxisAlignedViewRotation(ImAxis3D axis, bool positive_direction)
	{
		constexpr double pi = 3.14159265358979323846;
		constexpr double half_pi = pi * 0.5;
		constexpr ImPlot3DPoint x_axis(1.f, 0.f, 0.f);
		constexpr ImPlot3DPoint y_axis(0.f, 1.f, 0.f);
		constexpr ImPlot3DPoint z_axis(0.f, 0.f, 1.f);

		if (axis == ImAxis3D_X && positive_direction)
			return ImPlot3DQuat(half_pi, y_axis) * ImPlot3DQuat(-half_pi, x_axis); // Left(+x)
		else if (axis == ImAxis3D_X && !positive_direction)
			return ImPlot3DQuat(-half_pi, z_axis) * ImPlot3DQuat(-half_pi, y_axis); // Right(-x)
		else if (axis == ImAxis3D_Y && positive_direction)
			return ImPlot3DQuat(-half_pi, x_axis); // Rear(+y)
		else if (axis == ImAxis3D_Y && !positive_direction)
			return ImPlot3DQuat(pi, z_axis)* ImPlot3DQuat(half_pi, x_axis); // Front(-y)
		else if (axis == ImAxis3D_Z && positive_direction)
			return ImPlot3DQuat(pi, z_axis)* ImPlot3DQuat(pi, x_axis); // BOTTOM(+z)
		else if (axis == ImAxis3D_Z && !positive_direction)
			return ImPlot3DQuat(); // TOP(-z)

		return ImPlot3DQuat();
	}

	ImU32 radarPointToColor(float doppler, float power) const
	{
		const float doppler_span = m_doppler_max - m_doppler_min;
		const float power_span = m_power_max - m_power_min;
		const float color_t = doppler_span > 0.0f
			? std::clamp((doppler - m_doppler_min) / doppler_span, 0.0f, 1.0f)
			: 0.0f;
		const float alpha = power_span > 0.0f
			? std::clamp((power - m_power_min) / power_span, 0.0f, 1.0f)
			: 0.0f;
		const ImVec4 color(1.0f - color_t, 0.0f, color_t, alpha);
		
		return ImGui::ColorConvertFloat4ToU32(color);
	}

private: // Static Methods
	static void glfw_error_callback(int error, const char* description)
	{
		throw Exception("[glfw] error: " + std::to_string(error) + ": " + description);
	}

	static void scatter_getter(void* user, const void* point, ImPlot3DPoint* out_point, ImU32* out_color)
	{
		const auto& app = *reinterpret_cast<const MyApp*>(user);
		const auto& p = *reinterpret_cast<const retina::Point*>(point);

		*out_point = ImPlot3DPoint(p.x, p.y, p.z);
		*out_color = app.radarPointToColor(p.doppler, p.power);
	}

private: // UI States
	ImPlot3DQuat m_view_rotation = initial_rotation;
	float m_doppler_min = -0.5f;
	float m_doppler_max = 0.5f;
	float m_power_min = 0.0f;
	float m_power_max = 5000.0f;
	bool m_show_frustum = false;
	bool m_show_targets = true;
	bool m_show_xy = false;
	bool m_show_xz = false;
	bool m_show_yz = false;

	float m_sidebar_width = 360.0f;

private:
	GLFWwindow* m_window;
	asio::io_context m_io;
	std::thread m_io_thread;
	std::unique_ptr<retina::DeviceClient> m_client;
	DebugConsole m_console;
	std::ostream& m_log;
};

int main(int argc, char** argv)
{
	try
	{
		MyApp app;
		app.run();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[fatal] exception: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
