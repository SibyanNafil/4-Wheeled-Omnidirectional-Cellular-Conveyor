// main.cpp
// SDL2 + Dear ImGui
// PC  : OpenGL 2.1 + imgui_impl_opengl2
// RPi : OpenGL ES 2.0 + imgui_impl_opengl3 (glsl "#version 100")
// Updated: Catppuccin Mocha theme, resizable panels, colored logs,
//          improved I2C panel, Result visualization, command dropdown

#define USE_GLES2 1

#include <SDL.h>
#if defined(USE_GLES2)
#  include <SDL_opengles2.h>
#else
#  include <SDL_opengl.h>
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cfloat>
#include <cstdarg>
#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <queue>
#include <unordered_set>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <ctime>

#if defined(__linux__)
#  include <fcntl.h>
#  include <unistd.h>
#  include <errno.h>
#  include <sys/ioctl.h>
#  include <linux/i2c-dev.h>
#endif

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_internal.h"

#if defined(USE_GLES2)
#  include "imgui_impl_opengl3.h"
#else
#  include "imgui_impl_opengl2.h"
#endif

static constexpr int kI2CDefaultBus = 1;

// ============================================================
//  Utilities
// ============================================================
static float Clamp(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static int ClampInt(int v, int lo, int hi)      { return (v < lo) ? lo : (v > hi) ? hi : v; }

static std::string GetTimestamp()
{
    using namespace std::chrono;
    auto now    = system_clock::now();
    auto epoch  = now.time_since_epoch();
    auto ms_rem = duration_cast<milliseconds>(epoch) % 1000;
    std::time_t t = system_clock::to_time_t(now);
    struct tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d",
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                  static_cast<int>(ms_rem.count()));
    return std::string(buf);
}

#define LOG_I2C(st, fmt, ...) \
    (st).i2c.log.AddLog("[%s] " fmt, GetTimestamp().c_str(), ##__VA_ARGS__)

// ============================================================
//  I2C Backend  (Linux only)
// ============================================================
#if defined(__linux__)
static std::string I2CDevPath(int bus)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "/dev/i2c-%d", bus);
    return std::string(buf);
}

static bool I2C_Open(int bus, int& outFd, std::string& outErr)
{
    outErr.clear();
    if (outFd >= 0) return true;
    std::string dev = I2CDevPath(bus);
    int fd = ::open(dev.c_str(), O_RDWR);
    if (fd < 0) { outErr = "open(" + dev + ") failed: " + std::strerror(errno); return false; }
    outFd = fd;
    return true;
}

static void I2C_Close(int& fd) { if (fd >= 0) { ::close(fd); fd = -1; } }

static bool I2C_SetAddr(int fd, int addr7, std::string& outErr)
{
    outErr.clear();
    if (ioctl(fd, I2C_SLAVE, addr7) < 0) {
        char b[32]; std::snprintf(b, sizeof(b), "0x%02X", addr7);
        outErr = std::string("ioctl(I2C_SLAVE,") + b + ") failed: " + std::strerror(errno);
        return false;
    }
    return true;
}

static bool I2C_WriteByte(int fd, int addr7, uint8_t v, std::string& outErr)
{
    outErr.clear();
    if (!I2C_SetAddr(fd, addr7, outErr)) return false;
    uint8_t b = v;
    if (::write(fd, &b, 1) != 1) {
        char s[32]; std::snprintf(s, sizeof(s), "0x%02X", addr7);
        outErr = std::string("write to ") + s + " failed: " + std::strerror(errno);
        return false;
    }
    return true;
}

static bool I2C_ReadByte(int fd, int addr7, uint8_t& outVal, std::string& outErr)
{
    outErr.clear();
    if (!I2C_SetAddr(fd, addr7, outErr)) return false;
    uint8_t b = 0;
    if (::read(fd, &b, 1) != 1) {
        char s[32]; std::snprintf(s, sizeof(s), "0x%02X", addr7);
        outErr = std::string("read from ") + s + " failed: " + std::strerror(errno);
        return false;
    }
    outVal = b;
    return true;
}
#endif // __linux__

// ============================================================
//  Generic I2C address map
// ============================================================
static constexpr int kI2CAddrBase    = 0x01;
static constexpr int kI2CAddrMax7Bit = 0x77;

static inline bool XYToAddr(int x, int y, int cols, int rows, int& outAddr)
{
    if (x < 0 || x >= cols || y < 0 || y >= rows) return false;
    int a = kI2CAddrBase + x + cols * y;
    if (a > kI2CAddrMax7Bit) return false;
    outAddr = a;
    return true;
}

static inline bool AddrToXY(int addr, int cols, int rows, int& outX, int& outY)
{
    const int addrMax = kI2CAddrBase + cols * rows - 1;
    if (addr < kI2CAddrBase || addr > addrMax) return false;
    int idx = addr - kI2CAddrBase;
    outX = idx % cols;
    outY = idx / cols;
    return true;
}

// ============================================================
//  ParseI2CCmd
// ============================================================
static inline bool ParseI2CCmd(const char* s, uint8_t& outCmd)
{
    if (!s) return false;
    std::string t(s);
    while (!t.empty() && std::isspace((unsigned char)t.front())) t.erase(t.begin());
    while (!t.empty() && std::isspace((unsigned char)t.back()))  t.pop_back();
    if (t.empty()) return false;

    auto up = t;
    for (auto& c : up) c = (char)std::toupper((unsigned char)c);

    if (up == "MAJU")                                             { outCmd = 1; return true; }
    if (up == "KANAN_ATAS" || up == "KANANATAS")                  { outCmd = 2; return true; }
    if (up == "KANAN")                                            { outCmd = 3; return true; }
    if (up == "KANAN_BAWAH" || up == "KANANBAWAH")                { outCmd = 4; return true; }
    if (up == "MUNDUR")                                           { outCmd = 5; return true; }
    if (up == "KIRI_BAWAH" || up == "KIRIBAWAH")                  { outCmd = 6; return true; }
    if (up == "KIRI")                                             { outCmd = 7; return true; }
    if (up == "KIRI_ATAS" || up == "KIRIATAS")                    { outCmd = 8; return true; }
    if (up == "STOP" || up == "STOP_BRAKE" || up == "STOPBRAKE")  { outCmd = 9; return true; }

    char* e = nullptr;
    long v = std::strtol(t.c_str(), &e, 0);
    if (e == t.c_str()) return false;
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    outCmd = (uint8_t)v;
    return true;
}

// ============================================================
//  Canvas / font constants
// ============================================================
static constexpr float kFitMarginPx = 2.0f;
static constexpr float kAxisScale   = 0.34f, kAxisMinPx = 10.0f, kAxisMaxPx = 52.0f;
static constexpr float kPortScale   = 0.38f, kPortMinPx = 10.0f, kPortMaxPx = 56.0f;
static constexpr float kPadMinPx    = 10.0f;

// ============================================================
//  Catppuccin Mocha — IM_COL32 palette (for canvas drawing)
// ============================================================
static constexpr ImU32 kCatBase    = IM_COL32(30,  30,  46,  255);
static constexpr ImU32 kCatMantle  = IM_COL32(24,  24,  37,  255);
static constexpr ImU32 kCatSurf0   = IM_COL32(49,  50,  68,  255);
static constexpr ImU32 kCatSurf1   = IM_COL32(69,  71,  90,  255);
static constexpr ImU32 kCatSurf2   = IM_COL32(88,  91,  112, 255);
static constexpr ImU32 kCatOverlay = IM_COL32(108, 112, 134, 255);
static constexpr ImU32 kCatText    = IM_COL32(205, 214, 244, 255);
static constexpr ImU32 kCatBlue    = IM_COL32(137, 180, 250, 255);
static constexpr ImU32 kCatGreen   = IM_COL32(166, 227, 161, 255);
static constexpr ImU32 kCatRed     = IM_COL32(243, 139, 168, 255);
static constexpr ImU32 kCatYellow  = IM_COL32(249, 226, 175, 255);
static constexpr ImU32 kCatPeach   = IM_COL32(250, 179, 135, 255);
static constexpr ImU32 kCatMauve   = IM_COL32(203, 166, 247, 255);
static constexpr ImU32 kCatSky     = IM_COL32(137, 220, 235, 255);
static constexpr ImU32 kCatTeal    = IM_COL32(148, 226, 213, 255);
static constexpr ImU32 kCatGridBg  = IM_COL32(239, 241, 245, 255); // near-white grid bg

// Catppuccin ImVec4 (for ImGui style)
static const ImVec4 kV4Base    = ImVec4(0.118f, 0.118f, 0.180f, 1.0f);
static const ImVec4 kV4Mantle  = ImVec4(0.094f, 0.094f, 0.145f, 1.0f);
static const ImVec4 kV4Crust   = ImVec4(0.067f, 0.067f, 0.106f, 1.0f);
static const ImVec4 kV4Surf0   = ImVec4(0.192f, 0.196f, 0.267f, 1.0f);
static const ImVec4 kV4Surf1   = ImVec4(0.271f, 0.278f, 0.353f, 1.0f);
static const ImVec4 kV4Surf2   = ImVec4(0.345f, 0.357f, 0.439f, 1.0f);
static const ImVec4 kV4Overlay0= ImVec4(0.424f, 0.439f, 0.525f, 1.0f);
static const ImVec4 kV4Overlay1= ImVec4(0.498f, 0.518f, 0.612f, 1.0f);
static const ImVec4 kV4Text    = ImVec4(0.804f, 0.839f, 0.957f, 1.0f);
static const ImVec4 kV4Lavender= ImVec4(0.706f, 0.749f, 0.996f, 1.0f);
static const ImVec4 kV4Blue    = ImVec4(0.537f, 0.706f, 0.980f, 1.0f);
static const ImVec4 kV4Sapphire= ImVec4(0.455f, 0.780f, 0.925f, 1.0f);
static const ImVec4 kV4Sky     = ImVec4(0.537f, 0.863f, 0.922f, 1.0f);
static const ImVec4 kV4Teal    = ImVec4(0.580f, 0.886f, 0.835f, 1.0f);
static const ImVec4 kV4Green   = ImVec4(0.651f, 0.890f, 0.631f, 1.0f);
static const ImVec4 kV4Yellow  = ImVec4(0.976f, 0.886f, 0.686f, 1.0f);
static const ImVec4 kV4Peach   = ImVec4(0.980f, 0.702f, 0.529f, 1.0f);
static const ImVec4 kV4Maroon  = ImVec4(0.922f, 0.627f, 0.675f, 1.0f);
static const ImVec4 kV4Red     = ImVec4(0.953f, 0.545f, 0.659f, 1.0f);
static const ImVec4 kV4Mauve   = ImVec4(0.796f, 0.651f, 0.969f, 1.0f);

// ============================================================
//  AppLog  — with per-line color + category filter
// ============================================================
struct AppLog
{
    ImGuiTextBuffer Buf;
    bool AutoScroll = true, ScrollToBottom = false;

    // Category filters
    bool fErr   = true;
    bool fExec  = true;
    bool fProx  = true;
    bool fTx    = true;
    bool fScan  = true;
    bool fDijk  = true;
    bool fPrime = true;
    bool fInfo  = true;

    AppLog() { Clear(); }
    void Clear() { Buf.clear(); ScrollToBottom = true; }

    void AddLog(const char* fmt, ...) IM_FMTARGS(2) {
        va_list a; va_start(a, fmt); Buf.appendfv(fmt, a); va_end(a);
        ScrollToBottom = true;
    }

    // Identify tag in a log line (after the timestamp bracket)
    // Returns pointer to tag start (e.g. "[ERR]") or nullptr
    static const char* FindTag(const char* line, const char* lineEnd)
    {
        const char* p = line;
        // Skip first bracket [timestamp]
        while (p < lineEnd && *p != '[') p++;
        while (p < lineEnd && *p != ']') p++;
        if (p >= lineEnd) return nullptr;
        p++; // skip ']'
        if (p < lineEnd && *p == '[') return p;
        // Also handle space after timestamp
        if (p < lineEnd && *p == ' ') p++;
        if (p < lineEnd && *p == '[') return p;
        return nullptr;
    }

    static ImVec4 GetLineColor(const char* line, const char* lineEnd)
    {
        const char* tag = FindTag(line, lineEnd);
        if (!tag) return ImVec4(0.651f, 0.678f, 0.784f, 1.0f); // subtext0

        if      (strncmp(tag, "[ERR]",     5) == 0) return kV4Red;
        else if (strncmp(tag, "[EXEC]",    6) == 0) return kV4Blue;
        else if (strncmp(tag, "[DIJKSTRA]",10) == 0) return kV4Green;
        else if (strncmp(tag, "[PROX]",    6) == 0) return kV4Teal;
        else if (strncmp(tag, "[TX]",      4) == 0) return kV4Yellow;
        else if (strncmp(tag, "[SCAN]",    6) == 0) return kV4Peach;
        else if (strncmp(tag, "[PRIME]",   7) == 0) return kV4Mauve;
        else if (strncmp(tag, "[INFO]",    6) == 0) return kV4Overlay1;
        else if (strncmp(tag, "[I2C]",     5) == 0) return kV4Sapphire;
        return kV4Overlay1;
    }

    bool ShouldShowLine(const char* line, const char* lineEnd) const
    {
        const char* tag = FindTag(line, lineEnd);
        if (!tag) return fInfo;

        if      (strncmp(tag, "[ERR]",     5) == 0) return fErr;
        else if (strncmp(tag, "[EXEC]",    6) == 0) return fExec;
        else if (strncmp(tag, "[DIJKSTRA]",10) == 0) return fDijk;
        else if (strncmp(tag, "[PROX]",    6) == 0) return fProx;
        else if (strncmp(tag, "[TX]",      4) == 0) return fTx;
        else if (strncmp(tag, "[SCAN]",    6) == 0) return fScan;
        else if (strncmp(tag, "[PRIME]",   7) == 0) return fPrime;
        else if (strncmp(tag, "[INFO]",    6) == 0) return fInfo;
        return true;
    }

    void DrawFilterBar()
    {
        ImGui::TextUnformatted("Filter:");
        ImGui::SameLine();
        struct F { bool* flag; const char* lbl; ImVec4 col; };
        F filters[] = {
            {&fErr,   "ERR",   kV4Red     },
            {&fExec,  "EXEC",  kV4Blue    },
            {&fProx,  "PROX",  kV4Teal    },
            {&fTx,    "TX",    kV4Yellow  },
            {&fScan,  "SCAN",  kV4Peach   },
            {&fDijk,  "DIJK",  kV4Green   },
            {&fPrime, "PRIME", kV4Mauve   },
            {&fInfo,  "INFO",  kV4Overlay1},
        };
        for (auto& f : filters) {
            ImGui::PushStyleColor(ImGuiCol_Text,      *f.flag ? f.col : kV4Overlay0);
            ImGui::PushStyleColor(ImGuiCol_CheckMark, f.col);
            ImGui::Checkbox(f.lbl, f.flag);
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
        }
        ImGui::NewLine();
    }

    void DrawScrollingRegion(const char* id, ImVec2 sz)
    {
        ImGui::BeginChild(id, sz, false, ImGuiWindowFlags_HorizontalScrollbar);

        const char* bufBegin = Buf.begin();
        const char* bufEnd   = Buf.end();
        const char* lineStart = bufBegin;

        for (const char* p = bufBegin; p <= bufEnd; p++) {
            if (p == bufEnd || *p == '\n') {
                if (p > lineStart) {
                    if (ShouldShowLine(lineStart, p)) {
                        ImVec4 col = GetLineColor(lineStart, p);
                        ImGui::PushStyleColor(ImGuiCol_Text, col);
                        ImGui::TextUnformatted(lineStart, p);
                        ImGui::PopStyleColor();
                    }
                }
                lineStart = p + 1;
            }
        }

        if (ScrollToBottom && AutoScroll) ImGui::SetScrollHereY(1.0f);
        ScrollToBottom = false;
        ImGui::EndChild();
    }
};

// ============================================================
//  Input helpers
// ============================================================
static int DigitsOnlyFilter(ImGuiInputTextCallbackData* d) {
    if (d->EventChar < 256) { char c = (char)d->EventChar; if (c < '0' || c > '9') return 1; }
    return 0;
}

static bool InputPositiveInt(const char* label, int* value, char* buf, size_t buf_size, int minValue)
{
    ImGuiInputTextFlags fl = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCharFilter;
    bool commit = ImGui::InputText(label, buf, buf_size, fl, DigitsOnlyFilter)
                  || ImGui::IsItemDeactivatedAfterEdit();
    if (commit) {
        long v = std::strtol(buf, nullptr, 10);
        if (v < minValue) v = minValue;
        if (v > INT_MAX) v = INT_MAX;
        *value = (int)v;
        std::snprintf(buf, buf_size, "%d", *value);
        return true;
    }
    return false;
}

// ============================================================
//  Port
// ============================================================
enum class PortSide : int { Bottom = 0, Top, Left, Right };

struct PortConfig {
    bool     enabled = true;
    PortSide side    = PortSide::Bottom;
    int      index   = 0;
};

static bool SamePortPos(const PortConfig& a, const PortConfig& b)
{ return a.side == b.side && a.index == b.index; }

// ============================================================
//  Sub-struct 1: CanvasViewState
// ============================================================
struct CanvasViewState
{
    float  baseCellPx = 45.0f;
    float  zoom = 1.0f, zoomMin = 0.2f, zoomMax = 8.0f;
    ImVec2 pan  = {0.0f, 0.0f};

    float padLeft   = 70.0f, padRight  = 30.0f;
    float padTop    = 30.0f, padBottom = 70.0f;

    bool showCenters = true;
    bool showAxes    = true;
    bool showPorts   = true;
    bool requestFit  = true;
};

// ============================================================
//  Sub-struct 2: PathState
// ============================================================
struct PathState
{
    bool  found     = false;
    float costCm    = 0.0f;
    double computeMs = 0.0;
    std::vector<int> cells;

    bool  lastFound      = false;
    int   lastPathNodes = 0;
    float lastCostCm     = 0.0f;
    int   lastTurnCount  = 0;
    int   lastStartX = -1, lastStartY = -1;
    int   lastGoalX  = -1, lastGoalY  = -1;

    std::vector<int> lastCells;
};

// ============================================================
//  Sub-struct 3: I2CExecState
// ============================================================
struct I2CExecState
{
    enum class Phase : int {
        Idle = 0,
        WaitProxStart,
        DelayAfterProx,
        PrimeNodes,
        WaitProxTurn,
        DelayAfterTurn,
        WaitShutdown
    };

    struct TurnGate {
        int      pathIdx  = -1;
        int      addr     = 0;
        uint8_t  inCmd    = 9, outCmd = 9;
        uint8_t  lastProx = 0xFF;
        uint32_t lastPollMs = 0, detectMs = 0;
        bool     detected = false, done = false;
        uint8_t  confirmCount = 0;
    };

    bool  running = false;
    Phase phase   = Phase::Idle;

    int      entryAddr       = 0;
    uint8_t  entryCmd        = 9;
    uint32_t entryPollMs     = 20;
    uint32_t entryLastPollMs = 0;
    uint8_t  entryLastProx   = 0xFF;

    uint32_t afterProxDelayMs = 0;
    uint32_t phaseStartMs     = 0;

    uint32_t shutdownStartMs  = 0;
    static constexpr uint32_t kShutdownDelayMs = 5000;

    uint8_t  proxConfirmNeeded = 3;
    uint8_t  entryConfirmCount = 0;

    std::vector<int>      path;
    std::vector<int>      addr;
    std::vector<uint8_t>  outCmd;
    std::vector<int8_t>   nextDx, nextDy;
    std::vector<uint8_t>  gateMask, gateInCmd;
    std::vector<TurnGate> turnGates;
    int      turnActive = 0;
    uint32_t turnPollMs = 10;
};

// ============================================================
//  Sub-struct 4: I2CState
// ============================================================
struct I2CState
{
    bool connected   = false;
    int  bus         = kI2CDefaultBus;
    int  manualAddr  = 0x1;
    char txBuf[256]  = {};

#if defined(__linux__)
    int fd = -1;
    ~I2CState() { if (fd >= 0) { ::close(fd); fd = -1; } }
#endif

    std::vector<int> detectedAddrs;
    int selectedIdx = -1;

    AppLog log;

    bool     proxGridActive     = true;
    uint32_t proxGridPollMs     = 100;
    uint32_t proxGridLastPollMs = 0;
    std::vector<uint8_t> proxGridVal;
    std::vector<bool>    proxGridValid;

    void ResizeProxGrid(int cols, int rows) {
        int N = std::max(cols * rows, 1);
        proxGridVal.assign(N, 0);
        proxGridValid.assign(N, false);
    }

    bool     proxWatchActive  = false;
    uint32_t proxPollMs       = 80;
    uint32_t proxLastPollMs   = 0;
    std::vector<int>     proxWatchAddrs;
    std::vector<int>     proxWatchPathIdx;
    std::vector<uint8_t> proxLastVal;

    I2CExecState exec;
};

// ============================================================
//  GridCanvasState
// ============================================================
struct GridCanvasState
{
    int cols = 3;
    int rows = 3;

    PortConfig portIn, portOut;
    std::vector<unsigned char> obstacle;

    bool dimsBufInit = false;
    char colsBuf[16] = {}, rowsBuf[16] = {};

    CanvasViewState view;
    PathState       path;
    I2CState        i2c;

    GridCanvasState()
    {
        portIn  = {true, PortSide::Bottom, 0};
        portOut = {true, PortSide::Top,    cols - 1};
        obstacle.assign(cols * rows, 0);
        i2c.ResizeProxGrid(cols, rows);
        i2c.log.AddLog("[%s][INFO] Aplikasi siap. I2C bus default: /dev/i2c-%d\n",
                       GetTimestamp().c_str(), kI2CDefaultBus);
    }
};

// ============================================================
//  ClearPathResult
// ============================================================
static void ClearPathResult(GridCanvasState& st)
{
    st.path.found  = false;
    st.path.costCm = 0.0f;
    st.path.computeMs = 0.0;
    st.path.cells.clear();
    st.path.lastFound      = false;
    st.path.lastPathNodes  = 0;
    st.path.lastCostCm     = 0.0f;
    st.path.lastTurnCount  = 0;
    st.path.lastStartX = st.path.lastStartY = st.path.lastGoalX = st.path.lastGoalY = -1;
}

// ============================================================
//  Port helpers
// ============================================================
static int MaxIndexForSide(const GridCanvasState& st, PortSide s)
{
    return (s == PortSide::Top || s == PortSide::Bottom)
           ? std::max(st.cols - 1, 0)
           : std::max(st.rows - 1, 0);
}

static void ClampPort(GridCanvasState& st, PortConfig& p)
{ p.index = ClampInt(p.index, 0, MaxIndexForSide(st, p.side)); }

static PortConfig FindAlternativePortPos(const GridCanvasState& st,
                                          const PortConfig& cur,
                                          const PortConfig& forbidden)
{
    struct Cand { PortConfig cfg; int cost; };
    auto sP = [](PortSide a, PortSide b) { return (a == b) ? 0 : 10; };

    Cand best; best.cost = INT_MAX; best.cfg = cur;
    PortSide sides[4] = { PortSide::Bottom, PortSide::Top, PortSide::Left, PortSide::Right };

    for (PortSide s : sides) {
        int mx = MaxIndexForSide(st, s);
        for (int idx = 0; idx <= mx; idx++) {
            PortConfig c = cur; c.side = s; c.index = idx;
            if (SamePortPos(c, forbidden)) continue;
            int cost = sP(cur.side, s) + std::abs(cur.index - idx);
            if (s == cur.side) cost -= 2;
            if (cost < best.cost) { best.cost = cost; best.cfg = c; }
        }
    }
    return best.cfg;
}

static void EnsurePortsDistinct(GridCanvasState& st, bool preferIn)
{
    ClampPort(st, st.portIn); ClampPort(st, st.portOut);
    if (!SamePortPos(st.portIn, st.portOut)) return;
    if (preferIn) st.portOut = FindAlternativePortPos(st, st.portOut, st.portIn);
    else          st.portIn  = FindAlternativePortPos(st, st.portIn,  st.portOut);
}

static bool PortToInsideCell(const GridCanvasState& st, const PortConfig& p, int& ox, int& oy)
{
    int x = 0, y = 0;
    switch (p.side) {
    case PortSide::Bottom: x = p.index;     y = 0;           break;
    case PortSide::Top:    x = p.index;     y = st.rows - 1; break;
    case PortSide::Left:   x = 0;           y = p.index;     break;
    case PortSide::Right:  x = st.cols - 1; y = p.index;     break;
    default:               x = p.index;     y = 0;           break;
    }
    ox = ClampInt(x, 0, std::max(st.cols - 1, 0));
    oy = ClampInt(y, 0, std::max(st.rows - 1, 0));
    return true;
}

// ============================================================
//  Obstacle helpers
// ============================================================
static int Idx(const GridCanvasState& st, int x, int y) { return y * st.cols + x; }

static void ResizeObstaclePreserve(GridCanvasState& st, int oldCols, int oldRows)
{
    std::vector<unsigned char> n(st.cols * st.rows, 0);
    for (int y = 0; y < std::min(oldRows, st.rows); y++)
        for (int x = 0; x < std::min(oldCols, st.cols); x++)
            n[y * st.cols + x] = st.obstacle[y * oldCols + x];
    st.obstacle.swap(n);
}

// ============================================================
//  View helpers
// ============================================================
static ImVec2 PortCenterWorld(const GridCanvasState& st, const PortConfig& p)
{
    const float d = 0.5f;
    switch (p.side) {
    case PortSide::Bottom: return ImVec2(p.index + 0.5f, -d);
    case PortSide::Top:    return ImVec2(p.index + 0.5f, st.rows + d);
    case PortSide::Left:   return ImVec2(-d,             p.index + 0.5f);
    case PortSide::Right:  return ImVec2(st.cols + d,    p.index + 0.5f);
    default:               return ImVec2(p.index + 0.5f, -d);
    }
}

static void ComputeAutoPaddingForAxes(const GridCanvasState& st, float cellPx,
                                       float& oL, float& oR, float& oT, float& oB)
{
    float pL = kPadMinPx, pR = kPadMinPx, pT = kPadMinPx, pB = kPadMinPx;
    if (st.view.showAxes) {
        ImFont* f   = ImGui::GetFont();
        float   af  = Clamp(cellPx * kAxisScale, kAxisMinPx, kAxisMaxPx);
        float   m   = Clamp(af * 0.35f, 4.0f, 18.0f);
        auto TS = [&](int v) -> ImVec2 {
            char b[16]; std::snprintf(b, sizeof(b), "%d", v);
            return f->CalcTextSizeA(af, FLT_MAX, 0.0f, b);
        };
        ImVec2 t0 = TS(0), tC = TS(std::max(st.cols, 0)), tR = TS(std::max(st.rows, 0));
        float mW = t0.x; if (tC.x > mW) mW = tC.x; if (tR.x > mW) mW = tR.x;
        float mH = t0.y; if (tC.y > mH) mH = tC.y; if (tR.y > mH) mH = tR.y;
        pL = std::ceil(m + mW + m + 2.0f);
        pB = std::ceil(m + mH + m + 2.0f);
        pT = std::max(kPadMinPx, m + 2.0f);
        pR = std::max(kPadMinPx, m + 2.0f);
    }
    oL = pL; oR = pR; oT = pT; oB = pB;
}

static void FitViewToEverything(GridCanvasState& st, ImVec2 p0, ImVec2 p1)
{
    float cW = std::max(p1.x - p0.x, 1.0f), cH = std::max(p1.y - p0.y, 1.0f);
    int   cols = std::max(st.cols, 1), rows = std::max(st.rows, 1);
    float wW = (float)cols + 2.0f, wH = (float)rows + 2.0f;

    float pL = kPadMinPx, pR = kPadMinPx, pT = kPadMinPx, pB = kPadMinPx;
    for (int i = 0; i < 3; ++i) {
        float iW   = std::max(cW - pL - pR, 1.0f), iH = std::max(cH - pT - pB, 1.0f);
        float cf   = std::min(std::max(iW - 2 * kFitMarginPx, 1.0f) / wW,
                              std::max(iH - 2 * kFitMarginPx, 1.0f) / wH);
        float cPx  = st.view.baseCellPx * Clamp(cf / st.view.baseCellPx, st.view.zoomMin, st.view.zoomMax);
        ComputeAutoPaddingForAxes(st, cPx, pL, pR, pT, pB);
        st.view.zoom = Clamp(cf / st.view.baseCellPx, st.view.zoomMin, st.view.zoomMax);
    }
    st.view.padLeft = pL; st.view.padRight = pR; st.view.padTop = pT; st.view.padBottom = pB;

    {
        float iW = std::max(cW - pL - pR, 1.0f), iH = std::max(cH - pT - pB, 1.0f);
        float cf = std::min(std::max(iW - 2 * kFitMarginPx, 1.0f) / wW,
                            std::max(iH - 2 * kFitMarginPx, 1.0f) / wH);
        st.view.zoom = Clamp(cf / st.view.baseCellPx, st.view.zoomMin, st.view.zoomMax);
    }

    float  cPx = st.view.baseCellPx * st.view.zoom;
    ImVec2 iP0(p0.x + pL, p0.y + pT), iP1(p1.x - pR, p1.y - pB);
    ImVec2 iC((iP0.x + iP1.x) * 0.5f, (iP0.y + iP1.y) * 0.5f);
    float  cx = -0.5f + (cols + 1) * 0.5f, cy = -0.5f + (rows + 1) * 0.5f;
    st.view.pan.x = (iC.x - cx * cPx) - (p0.x + pL);
    st.view.pan.y = (iC.y + cy * cPx) - (p1.y - pB);
}

// ============================================================
//  Dijkstra
// ============================================================
static constexpr float kCostOrthoCm = 20.0f;
static constexpr float kCostDiagCm  = 28.284271247f;

struct DijkstraResult { bool found = false; float cost = 0; std::vector<int> path; };

static DijkstraResult RunDijkstraImplicit(const GridCanvasState& st,
                                           int startId, int goalId, bool pCC)
{
    DijkstraResult out;
    const int cols = st.cols, rows = st.rows, V = cols * rows;
    if (V <= 0) return out;

    const float INF = 1e30f;
    std::vector<float> dist(V, INF);
    std::vector<int>   prev(V, -1);

    using PQ = std::pair<float, int>;
    struct Cmp { bool operator()(const PQ& a, const PQ& b) const { return a.first > b.first; } };
    std::priority_queue<PQ, std::vector<PQ>, Cmp> pq;

    dist[startId] = 0;
    pq.push({0, startId});

    static const int D[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du != dist[u]) continue;
        if (u == goalId) break;
        if (st.obstacle[u]) continue;

        int ux = u % cols, uy = u / cols;
        for (int k = 0; k < 8; k++) {
            int dx = D[k][0], dy = D[k][1], vx = ux + dx, vy = uy + dy;
            if (vx < 0 || vx >= cols || vy < 0 || vy >= rows) continue;
            int v = vy * cols + vx;
            if (st.obstacle[v]) continue;
            if (pCC && dx != 0 && dy != 0 &&
                (st.obstacle[uy * cols + vx] || st.obstacle[vy * cols + ux])) continue;
            float w = (dx == 0 || dy == 0) ? kCostOrthoCm : kCostDiagCm;
            float nd = du + w;
            if (nd < dist[v]) { dist[v] = nd; prev[v] = u; pq.push({nd, v}); }
        }
    }

    if (dist[goalId] >= INF * 0.5f) return out;
    out.found = true; out.cost = dist[goalId];
    int cur = goalId;
    while (cur != -1) { out.path.push_back(cur); if (cur == startId) break; cur = prev[cur]; }
    std::reverse(out.path.begin(), out.path.end());
    return out;
}

static int CountTurns(const std::vector<int>& cells, int cols)
{
    int turns = 0;
    const int N = (int)cells.size();
    for (int i = 1; i < N - 1; ++i) {
        int dx0 = cells[i]   % cols - cells[i-1] % cols;
        int dy0 = cells[i]   / cols - cells[i-1] / cols;
        int dx1 = cells[i+1] % cols - cells[i]   % cols;
        int dy1 = cells[i+1] / cols - cells[i]   / cols;
        if (dx0 != dx1 || dy0 != dy1) turns++;
    }
    return turns;
}

static void RunSimulationDijkstra(GridCanvasState& st)
{
    ClearPathResult(st);
    int sx, sy, gx, gy;
    if (!PortToInsideCell(st, st.portIn, sx, sy) || !PortToInsideCell(st, st.portOut, gx, gy)) {
        st.i2c.log.AddLog("[%s][ERR] Port mapping error.\n", GetTimestamp().c_str());
        return;
    }
    int sId = Idx(st, sx, sy), gId = Idx(st, gx, gy);
    if (st.obstacle[sId]) {
        st.i2c.log.AddLog("[%s][ERR] Start (%d,%d) adalah OBSTACLE.\n", GetTimestamp().c_str(), sx, sy);
        return;
    }
    if (st.obstacle[gId]) {
        st.i2c.log.AddLog("[%s][ERR] Goal (%d,%d) adalah OBSTACLE.\n", GetTimestamp().c_str(), gx, gy);
        return;
    }

    st.i2c.log.AddLog("[%s][DIJKSTRA] Grid %dx%d | start=(%d,%d) goal=(%d,%d)\n",
                      GetTimestamp().c_str(), st.cols, st.rows, sx, sy, gx, gy);

    auto t0 = std::chrono::high_resolution_clock::now();
    DijkstraResult res = RunDijkstraImplicit(st, sId, gId, true);
    auto t1 = std::chrono::high_resolution_clock::now();

    st.path.computeMs   = std::chrono::duration<double, std::milli>(t1 - t0).count();
    st.path.lastStartX  = sx; st.path.lastStartY = sy;
    st.path.lastGoalX   = gx; st.path.lastGoalY  = gy;
    st.path.lastFound   = res.found;

    if (!res.found) {
        st.i2c.log.AddLog("[%s][DIJKSTRA] Path TIDAK ditemukan.  %.3f ms\n",
                          GetTimestamp().c_str(), st.path.computeMs);
        st.path.lastPathNodes = 0; st.path.lastCostCm = 0; st.path.lastTurnCount = 0;
        return;
    }

    st.path.found     = true;
    st.path.costCm    = res.cost;
    st.path.cells     = std::move(res.path);
    st.path.lastCells = st.path.cells;
    st.path.lastTurnCount = CountTurns(st.path.cells, st.cols);

    st.i2c.log.AddLog("[%s][DIJKSTRA] Path ditemukan: nodes=%d | turns=%d | cost=%.2f cm | %.3f ms\n",
                      GetTimestamp().c_str(),
                      (int)st.path.cells.size(), st.path.lastTurnCount,
                      st.path.costCm, st.path.computeMs);

    st.path.lastPathNodes = (int)st.path.cells.size();
    st.path.lastCostCm    = st.path.costCm;

    st.i2c.log.AddLog("[%s][DIJKSTRA] Path: ", GetTimestamp().c_str());
    for (size_t i = 0; i < st.path.cells.size(); i++) {
        int id = st.path.cells[i];
        st.i2c.log.AddLog("(%d,%d)%s", id % st.cols, id / st.cols,
                          (i + 1 < st.path.cells.size()) ? " -> " : "\n");
    }
}

// ============================================================
//  Drawing helpers
// ============================================================
static void DrawArrow(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thick, float hLen)
{
    ImVec2 d(b.x - a.x, b.y - a.y);
    float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-3f) return;
    ImVec2 dir(d.x / len, d.y / len), perp(-dir.y, dir.x);
    dl->AddLine(a, b, col, thick);
    float hw = hLen * 0.55f;
    ImVec2 tip = b;
    ImVec2 L(tip.x - dir.x * hLen + perp.x * hw, tip.y - dir.y * hLen + perp.y * hw);
    ImVec2 R(tip.x - dir.x * hLen - perp.x * hw, tip.y - dir.y * hLen - perp.y * hw);
    dl->AddTriangleFilled(tip, L, R, col);
}

// ============================================================
//  Apply Catppuccin Mocha Theme
// ============================================================
static void ApplyCatppuccinTheme()
{
    ImGuiStyle& style  = ImGui::GetStyle();
    ImVec4*     colors = style.Colors;

    colors[ImGuiCol_Text]                  = kV4Text;
    colors[ImGuiCol_TextDisabled]          = kV4Overlay0;
    colors[ImGuiCol_WindowBg]              = kV4Base;
    colors[ImGuiCol_ChildBg]               = kV4Mantle;
    colors[ImGuiCol_PopupBg]               = kV4Surf0;
    colors[ImGuiCol_Border]                = kV4Surf1;
    colors[ImGuiCol_BorderShadow]          = kV4Crust;
    colors[ImGuiCol_FrameBg]               = kV4Surf0;
    colors[ImGuiCol_FrameBgHovered]        = kV4Surf1;
    colors[ImGuiCol_FrameBgActive]         = kV4Surf2;
    colors[ImGuiCol_TitleBg]               = kV4Mantle;
    colors[ImGuiCol_TitleBgActive]         = kV4Surf0;
    colors[ImGuiCol_TitleBgCollapsed]      = kV4Crust;
    colors[ImGuiCol_MenuBarBg]             = kV4Mantle;
    colors[ImGuiCol_ScrollbarBg]           = kV4Mantle;
    colors[ImGuiCol_ScrollbarGrab]         = kV4Surf1;
    colors[ImGuiCol_ScrollbarGrabHovered]  = kV4Surf2;
    colors[ImGuiCol_ScrollbarGrabActive]   = kV4Overlay0;
    colors[ImGuiCol_CheckMark]             = kV4Green;
    colors[ImGuiCol_SliderGrab]            = kV4Blue;
    colors[ImGuiCol_SliderGrabActive]      = kV4Lavender;
    colors[ImGuiCol_Button]                = kV4Surf1;
    colors[ImGuiCol_ButtonHovered]         = kV4Surf2;
    colors[ImGuiCol_ButtonActive]          = kV4Overlay0;
    colors[ImGuiCol_Header]                = kV4Surf0;
    colors[ImGuiCol_HeaderHovered]         = kV4Surf1;
    colors[ImGuiCol_HeaderActive]          = kV4Surf2;
    colors[ImGuiCol_Separator]             = kV4Surf1;
    colors[ImGuiCol_SeparatorHovered]      = kV4Blue;
    colors[ImGuiCol_SeparatorActive]       = kV4Lavender;
    colors[ImGuiCol_ResizeGrip]            = kV4Surf1;
    colors[ImGuiCol_ResizeGripHovered]     = kV4Blue;
    colors[ImGuiCol_ResizeGripActive]      = kV4Lavender;
    colors[ImGuiCol_Tab]                   = kV4Surf0;
    colors[ImGuiCol_TabHovered]            = kV4Surf2;
    colors[ImGuiCol_TabActive]             = kV4Surf1;
    colors[ImGuiCol_TabUnfocused]          = kV4Mantle;
    colors[ImGuiCol_TabUnfocusedActive]    = kV4Surf0;
    colors[ImGuiCol_PlotLines]             = kV4Blue;
    colors[ImGuiCol_PlotLinesHovered]      = kV4Sapphire;
    colors[ImGuiCol_PlotHistogram]         = kV4Peach;
    colors[ImGuiCol_PlotHistogramHovered]  = kV4Yellow;
    colors[ImGuiCol_TableHeaderBg]         = kV4Surf0;
    colors[ImGuiCol_TableBorderStrong]     = kV4Surf1;
    colors[ImGuiCol_TableBorderLight]      = kV4Surf0;
    colors[ImGuiCol_TableRowBg]            = ImVec4(0,0,0,0);
    colors[ImGuiCol_TableRowBgAlt]         = ImVec4(kV4Surf0.x, kV4Surf0.y, kV4Surf0.z, 0.4f);
    colors[ImGuiCol_TextSelectedBg]        = ImVec4(kV4Blue.x, kV4Blue.y, kV4Blue.z, 0.35f);
    colors[ImGuiCol_DragDropTarget]        = kV4Yellow;
    colors[ImGuiCol_NavHighlight]          = kV4Lavender;
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1,1,1,0.7f);
    colors[ImGuiCol_NavWindowingDimBg]     = ImVec4(0.8f,0.8f,0.8f,0.2f);
    colors[ImGuiCol_ModalWindowDimBg]      = ImVec4(0.1f,0.1f,0.15f,0.5f);

    // Rounded, comfortable style
    style.WindowRounding    = 6.0f;
    style.ChildRounding     = 5.0f;
    style.FrameRounding     = 5.0f;
    style.PopupRounding     = 5.0f;
    style.ScrollbarRounding = 5.0f;
    style.GrabRounding      = 5.0f;
    style.TabRounding       = 5.0f;
    style.FramePadding      = ImVec2(9.0f, 6.0f);
    style.ItemSpacing       = ImVec2(9.0f, 7.0f);
    style.IndentSpacing     = 20.0f;
    style.ScrollbarSize     = 14.0f;
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 0.0f;
    style.TabBorderSize     = 0.0f;
    style.WindowPadding     = ImVec2(10.0f, 10.0f);
    style.CellPadding       = ImVec2(6.0f, 4.0f);
}

// ============================================================
//  Splitter helpers
// ============================================================
static void DrawVerticalSplitter(float& leftWidth, float minLeft, float maxLeft)
{
    const float kThk = 6.0f;
    ImGui::SameLine(0, 0);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImGui::GetStyleColorVec4(ImGuiCol_Separator));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_SeparatorHovered));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImGui::GetStyleColorVec4(ImGuiCol_SeparatorActive));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
    ImGui::Button("##vsplit", ImVec2(kThk, -1));
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemActive())
        leftWidth = Clamp(leftWidth + ImGui::GetIO().MouseDelta.x, minLeft, maxLeft);
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    ImGui::SameLine(0, 0);
}

static void DrawHorizontalSplitter(float& topHeight, float minTop, float maxTop)
{
    const float kThk = 6.0f;
    ImGui::PushStyleColor(ImGuiCol_Button,        ImGui::GetStyleColorVec4(ImGuiCol_Separator));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_SeparatorHovered));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImGui::GetStyleColorVec4(ImGuiCol_SeparatorActive));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
    ImGui::Button("##hsplit", ImVec2(-1, kThk));
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemActive())
        topHeight = Clamp(topHeight - ImGui::GetIO().MouseDelta.y, minTop, maxTop);
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
}

// ============================================================
//  Canvas
// ============================================================
static void DrawCanvasChild(GridCanvasState& st)
{
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    ImGui::BeginChild("##CC", ImVec2(0, 0), true,
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_MenuBar);
    ImGui::PopStyleColor();

    if (ImGui::BeginMenuBar()) {
        ImGui::PushStyleColor(ImGuiCol_PopupBg, kV4Surf0);
        if (ImGui::BeginMenu("Menu")) {
            ImGui::MenuItem("Show centers", nullptr, &st.view.showCenters);
            ImGui::MenuItem("Show axes",    nullptr, &st.view.showAxes);
            ImGui::MenuItem("Show ports",   nullptr, &st.view.showPorts);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset View (Fit)")) st.view.requestFit = true;
            ImGui::EndMenu();
        }
        ImGui::PopStyleColor();
        ImGui::EndMenuBar();
    }

    ImVec2 cs = ImGui::GetContentRegionAvail();
    if (cs.x < 50) cs.x = 50;
    if (cs.y < 50) cs.y = 50;
    ImVec2 p0 = ImGui::GetCursorScreenPos(), p1(p0.x + cs.x, p0.y + cs.y);

    ImGui::InvisibleButton("gc", cs,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);

    bool        hov = ImGui::IsItemHovered();
    ImDrawList* dl  = ImGui::GetWindowDrawList();
    ImGuiIO&    io  = ImGui::GetIO();

    // Canvas outer background (white)
    dl->AddRectFilled(p0, p1, IM_COL32(255, 255, 255, 255));
    dl->AddRect(p0, p1, IM_COL32(120, 120, 120, 255));
    dl->PushClipRect(p0, p1, true);

    auto CPx = [&]() -> float { return st.view.baseCellPx * st.view.zoom; };
    auto GBL = [&](float) -> ImVec2 {
        return ImVec2(p0.x + st.view.padLeft  + st.view.pan.x,
                      p1.y - st.view.padBottom + st.view.pan.y);
    };
    auto W2S = [&](ImVec2 w, float cPx, ImVec2 gBL) -> ImVec2 {
        return ImVec2(gBL.x + w.x * cPx, gBL.y - w.y * cPx);
    };
    auto S2W = [&](ImVec2 s, float cPx, ImVec2 gBL) -> ImVec2 {
        return ImVec2((s.x - gBL.x) / cPx, (gBL.y - s.y) / cPx);
    };

    if (st.view.requestFit) { FitViewToEverything(st, p0, p1); st.view.requestFit = false; }

    if (hov && (ImGui::IsMouseDragging(ImGuiMouseButton_Right) ||
                ImGui::IsMouseDragging(ImGuiMouseButton_Middle))) {
        st.view.pan.x += io.MouseDelta.x; st.view.pan.y += io.MouseDelta.y;
    }

    if (hov && io.MouseWheel != 0.0f) {
        float  oC = CPx(); ImVec2 oBL = GBL(oC), m = io.MousePos, wB = S2W(m, oC, oBL);
        float  nZ = Clamp(st.view.zoom * std::pow(1.1f, io.MouseWheel), st.view.zoomMin, st.view.zoomMax);
        if (nZ != st.view.zoom) {
            st.view.zoom = nZ;
            float nC = CPx(); ImVec2 nBL = GBL(nC), mA = W2S(wB, nC, nBL);
            st.view.pan.x += m.x - mA.x; st.view.pan.y += m.y - mA.y;
        }
    }

    float  cPx = CPx();
    ImVec2 gBL = GBL(cPx);
    ImVec2 gTL(gBL.x, gBL.y - st.rows * cPx), gBR(gBL.x + st.cols * cPx, gBL.y);
    ImVec2 fTL = W2S(ImVec2(-1.0f, (float)st.rows + 1.0f), cPx, gBL);
    ImVec2 fBR = W2S(ImVec2((float)st.cols + 1.0f, -1.0f), cPx, gBL);

    // Outer border area (gray)
    dl->AddRectFilled(fTL, fBR, IM_COL32(200, 200, 200, 255));
    dl->AddRect(fTL, fBR, IM_COL32(140, 140, 140, 255), 0.0f, 0, 2.0f);
    // Grid background (white)
    dl->AddRectFilled(gTL, gBR, IM_COL32(255, 255, 255, 255));

    // Obstacles
    for (int y = 0; y < st.rows; y++)
        for (int x = 0; x < st.cols; x++) {
            if (!st.obstacle[Idx(st, x, y)]) continue;
            ImVec2 a = W2S(ImVec2((float)x,   (float)y),   cPx, gBL);
            ImVec2 b = W2S(ImVec2((float)x+1, (float)y+1), cPx, gBL);
            dl->AddRectFilled(ImVec2(std::min(a.x,b.x), std::min(a.y,b.y)),
                              ImVec2(std::max(a.x,b.x), std::max(a.y,b.y)),
                              IM_COL32(220, 40, 40, 255));
            dl->AddRect(ImVec2(std::min(a.x,b.x), std::min(a.y,b.y)),
                        ImVec2(std::max(a.x,b.x), std::max(a.y,b.y)),
                        IM_COL32(180, 20, 20, 255), 0.0f, 0, 1.5f);
        }

    // Grid lines
    for (int x = 0; x <= st.cols; x++)
        dl->AddLine(W2S(ImVec2((float)x, 0),             cPx, gBL),
                    W2S(ImVec2((float)x, (float)st.rows), cPx, gBL),
                    IM_COL32(120, 120, 120, 255), 1.0f);
    for (int y = 0; y <= st.rows; y++)
        dl->AddLine(W2S(ImVec2(0,             (float)y), cPx, gBL),
                    W2S(ImVec2((float)st.cols, (float)y), cPx, gBL),
                    IM_COL32(120, 120, 120, 255), 1.0f);
    dl->AddRect(gTL, gBR, IM_COL32(60, 60, 60, 255), 0.0f, 0, 2.0f);

    // Highlight active exec nodes (executing path nodes)
    if (st.i2c.exec.running && !st.i2c.exec.path.empty()) {
        uint32_t ticks = SDL_GetTicks();
        float pulse = 0.55f + 0.45f * std::sin((float)ticks * 0.006f);
        ImU32 hlCol = IM_COL32((int)(255*pulse), (int)(220*pulse), (int)(50*pulse), 120);

        for (int id : st.i2c.exec.path) {
            int ex = id % st.cols, ey = id / st.cols;
            ImVec2 ca = W2S(ImVec2((float)ex,   (float)ey),   cPx, gBL);
            ImVec2 cb = W2S(ImVec2((float)ex+1, (float)ey+1), cPx, gBL);
            dl->AddRectFilled(
                ImVec2(std::min(ca.x,cb.x)+1, std::min(ca.y,cb.y)+1),
                ImVec2(std::max(ca.x,cb.x)-1, std::max(ca.y,cb.y)-1),
                hlCol);
        }
    }

    // Node centers
    if (st.view.showCenters) {
        float r = Clamp(cPx * 0.06f, 1.5f, 6.0f);
        for (int cx = 0; cx < st.cols; cx++)
            for (int cy = 0; cy < st.rows; cy++) {
                if (st.obstacle[Idx(st, cx, cy)]) continue;
                dl->AddCircleFilled(W2S(ImVec2(cx + 0.5f, cy + 0.5f), cPx, gBL), r, IM_COL32(0, 0, 0, 255));
            }
    }

    // Path
    if (st.path.found && !st.path.cells.empty()) {
        float  thick = Clamp(cPx * 0.10f, 1.5f, 6.0f);
        float  head  = Clamp(cPx * 0.25f, 5.0f, 20.0f);
        float  nR    = Clamp(cPx * 0.12f, 2.5f, 8.0f);

        std::vector<ImVec2> pts;
        // bool fP = false, lP = false;
        // if (st.view.showPorts) { pts.push_back(PortCenterWorld(st, st.portIn));  fP = true; }
        for (int id : st.path.cells) pts.push_back(ImVec2(id % st.cols + 0.5f, id / st.cols + 0.5f));
        // if (st.view.showPorts) { pts.push_back(PortCenterWorld(st, st.portOut)); lP = true; }

        for (int i = 0; i + 1 < (int)pts.size(); i++) {
            ImVec2 a = W2S(pts[i],   cPx, gBL), b = W2S(pts[i+1], cPx, gBL);
            ImVec2 d(b.x - a.x, b.y - a.y);
            float  len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len < 1e-3f) continue;
            ImVec2 dir(d.x / len, d.y / len);
            float sA = nR * 0.35f;
            float sB = nR * 1.15f;
            ImVec2 a2(a.x + dir.x * sA, a.y + dir.y * sA), b2(b.x - dir.x * sB, b.y - dir.y * sB);
            ImVec2 dd(b2.x - a2.x, b2.y - a2.y);
            if (dd.x * dd.x + dd.y * dd.y < 4.0f) { a2 = a; b2 = b; }
            DrawArrow(dl, a2, b2, IM_COL32(0, 90, 255, 255), thick, head);
        }
        for (int id : st.path.cells)
            dl->AddCircleFilled(W2S(ImVec2(id % st.cols + 0.5f, id / st.cols + 0.5f), cPx, gBL),
                                nR, IM_COL32(0, 0, 0, 255));
    }

    // Ports
    auto DP = [&](const PortConfig& p, ImU32 fc, ImU32 tc, const char* lbl) {
        if (!st.view.showPorts) return;
        ImVec2 sc = W2S(PortCenterWorld(st, p), cPx, gBL);
        float  h  = cPx * 0.5f;
        float  rnd = Clamp(cPx * 0.08f, 2.0f, 8.0f);
        ImVec2 rm(sc.x - h, sc.y - h), rx(sc.x + h, sc.y + h);
        dl->AddRectFilled(rm, rx, fc, rnd);
        dl->AddRect(rm, rx, IM_COL32(20,20,20,200), rnd, 0, 1.5f);
        ImFont* f  = ImGui::GetFont();
        float   fz = Clamp(cPx * kPortScale, kPortMinPx, kPortMaxPx);
        ImVec2  ts = f->CalcTextSizeA(fz, FLT_MAX, 0.0f, lbl);
        dl->AddText(f, fz, ImVec2(sc.x - ts.x * 0.5f, sc.y - ts.y * 0.5f), tc, lbl);
    };
    DP(st.portIn,  IM_COL32(0, 255, 0, 255),   IM_COL32(0, 0, 0, 255), "IN");
    DP(st.portOut, IM_COL32(0, 255, 255, 255), IM_COL32(0, 0, 0, 255), "OUT");

    // Axes
    if (st.view.showAxes) {
        ImFont* f   = ImGui::GetFont();
        float   af  = Clamp(cPx * kAxisScale, kAxisMinPx, kAxisMaxPx);
        float   m   = Clamp(af * 0.35f, 3.0f, 14.0f);
        float   tY  = fBR.y + m, tXB = fTL.x - m;
        for (int x = 0; x <= st.cols; x++) {
            ImVec2 pp = W2S(ImVec2((float)x, 0), cPx, gBL);
            char b[16]; std::snprintf(b, sizeof(b), "%d", x);
            ImVec2 ts = f->CalcTextSizeA(af, FLT_MAX, 0.0f, b);
            dl->AddText(f, af, ImVec2(pp.x - ts.x * 0.5f, tY), IM_COL32(0, 0, 0, 255), b);
        }
        for (int y = 0; y <= st.rows; y++) {
            ImVec2 pp = W2S(ImVec2(0, (float)y), cPx, gBL);
            char b[16]; std::snprintf(b, sizeof(b), "%d", y);
            ImVec2 ts = f->CalcTextSizeA(af, FLT_MAX, 0.0f, b);
            dl->AddText(f, af, ImVec2(tXB - ts.x, pp.y - ts.y * 0.5f), IM_COL32(0, 0, 0, 255), b);
        }
    }

    if (hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImVec2 w = S2W(io.MousePos, cPx, gBL);
        int cx = (int)std::floor(w.x), cy = (int)std::floor(w.y);
        if (cx >= 0 && cx < st.cols && cy >= 0 && cy < st.rows) {
            st.obstacle[Idx(st, cx, cy)] ^= 1;
            if (st.path.found) {
                ClearPathResult(st);
                st.i2c.log.AddLog("[%s][INFO] Obstacle (%d,%d) diubah -> path direset.\n",
                                  GetTimestamp().c_str(), cx, cy);
            }
        }
    }

    if (hov) {
        ImVec2 w = S2W(io.MousePos, cPx, gBL);
        int cx = (int)std::floor(w.x), cy = (int)std::floor(w.y);
        if (cx >= 0 && cx < st.cols && cy >= 0 && cy < st.rows) {
            ImGui::BeginTooltip();
            ImGui::Text("Cell (%d, %d)", cx, cy);
            bool isObs = st.obstacle[Idx(st, cx, cy)] != 0;
            ImGui::Text("Obstacle: %s", isObs ? "YES" : "NO");
            int addr = 0;
            if (XYToAddr(cx, cy, st.cols, st.rows, addr))
                ImGui::Text("I2C Addr: 0x%02X", addr);
            ImGui::TextDisabled("Left-click: toggle obstacle");
            ImGui::EndTooltip();
        }
    }

    dl->PopClipRect();
    ImGui::EndChild();
}

// ============================================================
//  Left panel — Simulation
// ============================================================
static void DrawLeftPanel_Simulation(GridCanvasState& st)
{
    // Run / Reset buttons
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Green.x*0.5f, kV4Green.y*0.5f, kV4Green.z*0.5f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Green.x*0.7f, kV4Green.y*0.7f, kV4Green.z*0.7f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Green);
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.05f, 0.05f, 0.05f, 1.0f));
    if (ImGui::Button("  Run Dijkstra  ")) RunSimulationDijkstra(st);
    ImGui::PopStyleColor(4);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button,        kV4Surf1);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kV4Surf2);
    if (ImGui::Button("Reset Path")) {
        ClearPathResult(st);
        st.i2c.log.AddLog("[%s][INFO] Path direset oleh user.\n", GetTimestamp().c_str());
    }
    ImGui::PopStyleColor(2);

    if (!st.dimsBufInit) {
        std::snprintf(st.colsBuf, sizeof(st.colsBuf), "%d", std::max(st.cols, 1));
        std::snprintf(st.rowsBuf, sizeof(st.rowsBuf), "%d", std::max(st.rows, 1));
        st.dimsBufInit = true;
    }

    int  oldCols = st.cols, oldRows = st.rows;
    bool dc = false;

    ImGui::SeparatorText("Grid");
    ImGui::SetNextItemWidth(110);
    if (InputPositiveInt("Cols##c", &st.cols, st.colsBuf, sizeof(st.colsBuf), 1)) dc = true;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110);
    if (InputPositiveInt("Rows##r", &st.rows, st.rowsBuf, sizeof(st.rowsBuf), 1)) dc = true;

    if (dc) {
        ResizeObstaclePreserve(st, oldCols, oldRows);
        st.i2c.ResizeProxGrid(st.cols, st.rows);
        ClampPort(st, st.portIn); ClampPort(st, st.portOut);
        EnsurePortsDistinct(st, true);
        st.view.requestFit = true;
        ClearPathResult(st);
        st.i2c.log.AddLog("[%s][INFO] Grid diubah: %dx%d -> %dx%d\n",
                          GetTimestamp().c_str(), oldCols, oldRows, st.cols, st.rows);
    }

    // Port IN
    ImGui::SeparatorText("Port IN");
    {
        bool ch = false; ImGui::PushID("PIN");
        int s = (int)st.portIn.side;
        ImGui::SetNextItemWidth(-1);
        if (ImGui::Combo("##Side_i", &s, "Bottom\0Top\0Left\0Right\0"))
            { st.portIn.side = (PortSide)s; ClampPort(st, st.portIn); ch = true; }
        int mx = std::max(MaxIndexForSide(st, st.portIn.side), 0);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##index_i", &st.portIn.index, 0, mx))
            { ClampPort(st, st.portIn); ch = true; }
        if (ch) { EnsurePortsDistinct(st, true); st.view.requestFit = true; ClearPathResult(st); }
        ImGui::PopID();
    }

    // Port OUT
    ImGui::SeparatorText("Port OUT");
    {
        bool ch = false; ImGui::PushID("POUT");
        int s = (int)st.portOut.side;
        ImGui::SetNextItemWidth(-1);
        if (ImGui::Combo("##Side_o", &s, "Bottom\0Top\0Left\0Right\0"))
            { st.portOut.side = (PortSide)s; ClampPort(st, st.portOut); ch = true; }
        int mx = std::max(MaxIndexForSide(st, st.portOut.side), 0);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##Index_o", &st.portOut.index, 0, mx))
            { ClampPort(st, st.portOut); ch = true; }
        if (ch) { EnsurePortsDistinct(st, false); st.view.requestFit = true; ClearPathResult(st); }
        ImGui::PopID();
    }

    ImGui::SeparatorText("Obstacles");
    int cnt = 0; for (unsigned char v : st.obstacle) cnt += (v != 0);
    ImGui::TextColored(kV4Overlay1, "Jumlah: %d", cnt);
    if (ImGui::Button("Clear All Obstacles")) {
        std::fill(st.obstacle.begin(), st.obstacle.end(), 0);
        ClearPathResult(st);
        st.i2c.log.AddLog("[%s][INFO] Semua obstacle dihapus.\n", GetTimestamp().c_str());
    }
}

// ============================================================
//  I2C Execution helpers (unchanged logic)
// ============================================================
static void ResetI2CExecState(GridCanvasState& st)
{
    I2CExecState& e = st.i2c.exec;
    e.running = false;
    e.phase   = I2CExecState::Phase::Idle;
    e.entryAddr = 0; e.entryCmd = 9;
    e.entryLastPollMs = 0; e.entryLastProx = 0xFF;
    e.phaseStartMs = 0;
    e.shutdownStartMs = 0;
    e.path.clear();    e.addr.clear();
    e.outCmd.clear();  e.nextDx.clear();  e.nextDy.clear();
    e.gateMask.clear(); e.gateInCmd.clear();
    e.turnGates.clear(); e.turnActive = 0;
    e.entryConfirmCount = 0;

    st.i2c.proxWatchActive = false;
    st.i2c.proxWatchAddrs.clear();
    st.i2c.proxWatchPathIdx.clear();
    st.i2c.proxLastVal.clear();
}

static uint8_t CmdFromDelta(int dx, int dy)
{
    if (dx ==  0 && dy == +1) return 1;
    if (dx == +1 && dy == +1) return 2;
    if (dx == +1 && dy ==  0) return 3;
    if (dx == +1 && dy == -1) return 4;
    if (dx ==  0 && dy == -1) return 5;
    if (dx == -1 && dy == -1) return 6;
    if (dx == -1 && dy ==  0) return 7;
    if (dx == -1 && dy == +1) return 8;
    return 9;
}

static bool DiagonalAssistCmds(int dx, int dy, uint8_t& cNX, uint8_t& cNY)
{
    if (dx == +1 && dy == +1) { cNX = 10; cNY = 14; return true; }
    if (dx == -1 && dy == +1) { cNX = 12; cNY = 16; return true; }
    if (dx == +1 && dy == -1) { cNX = 17; cNY = 13; return true; }
    if (dx == -1 && dy == -1) { cNX = 15; cNY = 11; return true; }
    return false;
}

static uint8_t CmdEnterFromPortInSide(PortSide s)
{
    switch (s) {
    case PortSide::Bottom: return 1; case PortSide::Left:  return 3;
    case PortSide::Top:    return 5; case PortSide::Right: return 7;
    default:               return 1;
    }
}

static uint8_t CmdEjectFromPortOutSide(PortSide s)
{
    switch (s) {
    case PortSide::Top:    return CmdFromDelta( 0, +1);
    case PortSide::Bottom: return CmdFromDelta( 0, -1);
    case PortSide::Left:   return CmdFromDelta(-1,  0);
    case PortSide::Right:  return CmdFromDelta(+1,  0);
    default:               return 9;
    }
}

static bool PrepareExecPlan_TurnGated(GridCanvasState& st)
{
    I2CExecState& e = st.i2c.exec;
    e.path = st.path.cells;
    const int N = (int)e.path.size();
    if (N <= 0) return false;

    e.addr.assign(N, 0); e.outCmd.assign(N, 9);
    e.nextDx.assign(N, 0); e.nextDy.assign(N, 0);
    e.gateMask.assign(N, 0); e.gateInCmd.assign(N, 9);
    e.turnGates.clear(); e.turnActive = 0;

    for (int i = 0; i < N; ++i) {
        int id = e.path[i], x = id % st.cols, y = id / st.cols, addr = 0;
        if (!XYToAddr(x, y, st.cols, st.rows, addr)) return false;
        e.addr[i] = addr;
    }
    for (int i = 0; i < N - 1; ++i) {
        int a = e.path[i], b = e.path[i + 1];
        int dx = b % st.cols - a % st.cols, dy = b / st.cols - a / st.cols;
        e.nextDx[i] = (int8_t)dx; e.nextDy[i] = (int8_t)dy;
        e.outCmd[i] = CmdFromDelta(dx, dy);
    }
    e.outCmd[N - 1] = CmdEjectFromPortOutSide(st.portOut.side);

    auto AddGateIfTurn = [&](int i) {
        int p = e.path[i - 1], c = e.path[i];
        uint8_t in  = CmdFromDelta(c % st.cols - p % st.cols, c / st.cols - p / st.cols);
        uint8_t out = e.outCmd[i];
        if (in == out) return;
        e.gateMask[i] = 1; e.gateInCmd[i] = in;
        I2CExecState::TurnGate g;
        g.pathIdx = i; g.addr = e.addr[i]; g.inCmd = in; g.outCmd = out;
        e.turnGates.push_back(g);
    };

    if (N >= 2) {
        for (int i = 1; i <= N - 2; ++i) AddGateIfTurn(i);
        {
            int i  = N - 1;
            int p  = e.path[N - 2], c = e.path[N - 1];
            uint8_t in  = CmdFromDelta(c % st.cols - p % st.cols, c / st.cols - p / st.cols);
            uint8_t out = e.outCmd[i];
            e.gateMask[i] = 1; e.gateInCmd[i] = in;
            I2CExecState::TurnGate g;
            g.pathIdx = i; g.addr = e.addr[i]; g.inCmd = in; g.outCmd = out;
            e.turnGates.push_back(g);
        }
    }
    return true;
}

static void BuildProxWatchList_StartAndTurns(GridCanvasState& st)
{
    I2CState& i2c = st.i2c;
    i2c.proxWatchAddrs.clear(); i2c.proxWatchPathIdx.clear();
    i2c.proxLastVal.clear(); i2c.proxLastPollMs = 0;
    if (!st.path.found || st.path.cells.empty()) return;

    const int N = (int)st.path.cells.size();
    std::unordered_set<int> seen;

    auto Add = [&](int i) {
        if (i < 0 || i >= N) return;
        int id = st.path.cells[i], x = id % st.cols, y = id / st.cols, addr = 0;
        if (!XYToAddr(x, y, st.cols, st.rows, addr)) return;
        if (seen.insert(addr).second) {
            i2c.proxWatchAddrs.push_back(addr);
            i2c.proxWatchPathIdx.push_back(i);
            i2c.proxLastVal.push_back(0xFF);
        }
    };

    Add(0);
    for (int i = 1; i <= N - 2; ++i) {
        int a0 = st.path.cells[i-1], a1 = st.path.cells[i], a2 = st.path.cells[i+1];
        if ((a1%st.cols - a0%st.cols) != (a2%st.cols - a1%st.cols) ||
            (a1/st.cols - a0/st.cols) != (a2/st.cols - a1/st.cols))
            Add(i);
    }
}

static void SendDiagonalAssistForPathIdx(GridCanvasState& st, int pathIdx,
                                          const char* tag, bool)
{
#if !defined(__linux__)
    (void)st; (void)pathIdx; (void)tag; return;
#else
    I2CExecState& e = st.i2c.exec;
    const int N = (int)e.path.size();
    if (!st.i2c.connected || st.i2c.fd < 0) return;
    if (pathIdx < 0 || pathIdx >= N - 1) return;

    const int dx = (int)e.nextDx[pathIdx], dy = (int)e.nextDy[pathIdx];
    if (dx == 0 || dy == 0) return;

    uint8_t cNX = 0, cNY = 0;
    if (!DiagonalAssistCmds(dx, dy, cNX, cNY)) return;

    const int aId = e.path[pathIdx], ax = aId % st.cols, ay = aId / st.cols;
    struct Cand { int x, y; uint8_t cmd; const char* name; };
    Cand cands[2] = { {ax + dx, ay, cNX, "NX"}, {ax, ay + dy, cNY, "NY"} };

    for (auto& c : cands) {
        int nx = c.x, ny = c.y;
        if (nx < 0 || nx >= st.cols || ny < 0 || ny >= st.rows) continue;
        if (!st.obstacle.empty() && st.obstacle[Idx(st, nx, ny)]) continue;
        int addrN = 0;
        if (!XYToAddr(nx, ny, st.cols, st.rows, addrN)) continue;
        std::string err;
        if (!I2C_WriteByte(st.i2c.fd, addrN, c.cmd, err)) {
            st.i2c.log.AddLog("[%s][ERR][ASSIST] %s %s addr=0x%02X cmd=%u: %s\n",
                              GetTimestamp().c_str(), tag, c.name,
                              addrN, (unsigned)c.cmd, err.c_str());
        }
    }
#endif
}

static bool PrimeAllPathNodes_TurnGated(GridCanvasState& st)
{
#if !defined(__linux__)
    (void)st; return false;
#else
    I2CExecState& e = st.i2c.exec;
    if (!st.i2c.connected || st.i2c.fd < 0) return false;
    if (e.addr.empty()) return false;

    auto W = [&](int addr, uint8_t cmd) -> bool {
        std::string err;
        if (!I2C_WriteByte(st.i2c.fd, addr, cmd, err)) {
            int x = 0, y = 0; AddrToXY(addr, st.cols, st.rows, x, y);
            st.i2c.log.AddLog("[%s][ERR][PRIME] TX gagal addr=0x%02X (%d,%d) cmd=%u: %s\n",
                              GetTimestamp().c_str(), addr, x, y, (unsigned)cmd, err.c_str());
            return false;
        }
        return true;
    };

    const int N = (int)e.addr.size();
    {
        int x = 0, y = 0; AddrToXY(e.addr[0], st.cols, st.rows, x, y);
        if (!W(e.addr[0], e.entryCmd)) return false;
        st.i2c.log.AddLog("[%s][PRIME] Node 0/%d addr=0x%02X (%d,%d) entryCmd=%u\n",
                          GetTimestamp().c_str(), N - 1, e.addr[0], x, y, (unsigned)e.entryCmd);
    }
    SendDiagonalAssistForPathIdx(st, 0, "PRIME", true);

    for (int i = 1; i < N; ++i) {
        uint8_t cmd = e.outCmd[i];
        if (!e.gateMask.empty() && e.gateMask[i]) cmd = e.gateInCmd[i];
        int x = 0, y = 0; AddrToXY(e.addr[i], st.cols, st.rows, x, y);
        if (!W(e.addr[i], cmd)) return false;
        st.i2c.log.AddLog("[%s][PRIME] Node %d/%d addr=0x%02X (%d,%d) cmd=%u%s\n",
                          GetTimestamp().c_str(), i, N - 1, e.addr[i], x, y,
                          (unsigned)cmd, e.gateMask[i] ? " [GATE]" : "");
        SendDiagonalAssistForPathIdx(st, i, "PRIME", true);
    }

    st.i2c.log.AddLog("[%s][PRIME] Selesai: %d node di-prime.\n", GetTimestamp().c_str(), N);
    return true;
#endif
}

// ============================================================
//  TickI2CProxGrid_AllNodes
// ============================================================
static void TickI2CProxGrid_AllNodes(GridCanvasState& st)
{
#if !defined(__linux__)
    (void)st; return;
#else
    I2CState& i2c = st.i2c;
    if (!i2c.proxGridActive) return;
    const uint32_t now = (uint32_t)SDL_GetTicks();
    if (i2c.proxGridLastPollMs != 0 && (now - i2c.proxGridLastPollMs) < i2c.proxGridPollMs) return;
    i2c.proxGridLastPollMs = now;
    if (!i2c.connected || i2c.fd < 0) return;

    const int total = st.cols * st.rows;
    if (total <= 0) return;

    for (int idx = 0; idx < total; ++idx) {
        int x = idx % st.cols, y = idx / st.cols, addr = 0;
        if (!XYToAddr(x, y, st.cols, st.rows, addr)) continue;
        uint8_t v = 0; std::string err;
        if (I2C_ReadByte(i2c.fd, addr, v, err)) {
            i2c.proxGridVal[idx]   = v;
            i2c.proxGridValid[idx] = true;
        } else {
            i2c.proxGridValid[idx] = false;
        }
    }
#endif
}

// ============================================================
//  Left panel — I2C  (Major rewrite)
// ============================================================
static void DrawLeftPanel_I2C(GridCanvasState& st)
{
#if !defined(__linux__)
    ImGui::TextColored(kV4Peach, "I2C hanya tersedia di Linux/Raspberry Pi.");
    ImGui::TextColored(kV4Overlay0, "Build Windows: fitur non-aktif.");
    ImGui::Spacing();
    ImGui::SeparatorText("Detected Nodes");
    if (ImGui::BeginListBox("##nodes", ImVec2(-FLT_MIN, 60)))
        ImGui::EndListBox();
    return;
#else
    I2CState&     i2c = st.i2c;
    I2CExecState& exe = st.i2c.exec;

    const int sF = kI2CAddrBase, sL = kI2CAddrBase + st.cols * st.rows - 1;
    const bool addrRangeOk = (sL <= kI2CAddrMax7Bit);

    // ---- Connection Status ----
    ImGui::SeparatorText("Status I2C");
    {
        // Status dot + label
        if (i2c.connected) {
            ImGui::TextColored(kV4Green, "[ON]");
            ImGui::SameLine();
            ImGui::TextColored(kV4Text, "/dev/i2c-%d", kI2CDefaultBus);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Red.x*0.4f, kV4Red.y*0.4f, kV4Red.z*0.4f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Red.x*0.6f, kV4Red.y*0.6f, kV4Red.z*0.6f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Red);
            if (ImGui::Button("Disconnect")) {
                I2C_Close(i2c.fd);
                i2c.connected = false;
                i2c.proxGridLastPollMs = 0;
                std::fill(i2c.proxGridValid.begin(), i2c.proxGridValid.end(), false);
                i2c.detectedAddrs.clear();
                i2c.selectedIdx = -1;
                i2c.log.AddLog("[%s][I2C] Disconnected.\n", GetTimestamp().c_str());
            }
            ImGui::PopStyleColor(3);
        } else {
            ImGui::TextColored(kV4Red, "[OFF]");
            ImGui::SameLine();
            ImGui::TextColored(kV4Overlay0, "/dev/i2c-%d", kI2CDefaultBus);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Green.x*0.4f, kV4Green.y*0.4f, kV4Green.z*0.4f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Green.x*0.6f, kV4Green.y*0.6f, kV4Green.z*0.6f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Green);
            ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.05f,0.05f,0.05f,1.0f));
            if (ImGui::Button("Connect")) {
                std::string err;
                if (I2C_Open(i2c.bus, i2c.fd, err)) {
                    i2c.connected = true;
                    i2c.log.AddLog("[%s][I2C] Terhubung: /dev/i2c-%d\n",
                                   GetTimestamp().c_str(), kI2CDefaultBus);
                } else {
                    i2c.log.AddLog("[%s][ERR][I2C] Gagal: %s\n",
                                   GetTimestamp().c_str(), err.c_str());
                }
            }
            ImGui::PopStyleColor(4);
        }
    }

    // ---- Node Discovery ----
    ImGui::SeparatorText("Scan All Conveyor Nodes");
    {
        char scanLbl[64];
        std::snprintf(scanLbl, sizeof(scanLbl), "SCAN");
        if (ImGui::Button(scanLbl)) {
            i2c.detectedAddrs.clear(); i2c.selectedIdx = -1;
            std::string err;
            if (!i2c.connected) {
                if (!I2C_Open(i2c.bus, i2c.fd, err)) {
                    i2c.log.AddLog("[%s][ERR][SCAN] Tidak dapat membuka I2C: %s\n",
                                   GetTimestamp().c_str(), err.c_str());
                    goto scan_done;
                }
                i2c.connected = true;
            }
            i2c.log.AddLog("[%s][SCAN] Scan addr 0x%02X..0x%02X (grid %dx%d)...\n",
                           GetTimestamp().c_str(), sF, sL, st.cols, st.rows);
            for (int addr = sF; addr <= sL; ++addr) {
                uint8_t v = 0;
                if (I2C_ReadByte(i2c.fd, addr, v, err)) {
                    i2c.detectedAddrs.push_back(addr);
                    int x = 0, y = 0; AddrToXY(addr, st.cols, st.rows, x, y);
                    i2c.log.AddLog("[%s][SCAN] Ditemukan: addr=0x%02X (x=%d,y=%d) prox=%d\n",
                                   GetTimestamp().c_str(), addr, x, y, (int)v);
                }
            }
            i2c.log.AddLog("[%s][SCAN] Selesai: %d/%d node ditemukan.\n",
                           GetTimestamp().c_str(),
                           (int)i2c.detectedAddrs.size(), st.cols * st.rows);
            scan_done:;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scan range: 0x%02X .. 0x%02X\nGrid: %dx%d (%d node)",
                              sF, sL, st.cols, st.rows, st.cols * st.rows);

        // Node summary
        if (!i2c.detectedAddrs.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(kV4Green, "%d / %d",
                               (int)i2c.detectedAddrs.size(), st.cols * st.rows);
        }

        // Listbox
        float lbH = i2c.detectedAddrs.empty() ? ImGui::GetTextLineHeightWithSpacing() * 2.5f : 90.0f;
        if (ImGui::BeginListBox("##nodes", ImVec2(-FLT_MIN, lbH))) {
            if (i2c.detectedAddrs.empty()) {
                ImGui::TextColored(kV4Overlay0,
                    i2c.connected ? "Belum scan / tidak ada node." : "Tidak terhubung.");
            } else {
                for (int i = 0; i < (int)i2c.detectedAddrs.size(); ++i) {
                    int addr = i2c.detectedAddrs[i], x = 0, y = 0;
                    AddrToXY(addr, st.cols, st.rows, x, y);
                    char lbl[64]; std::snprintf(lbl, sizeof(lbl), "0x%02X  (%d,%d)", addr, x, y);
                    bool sel = (i2c.selectedIdx == i);
                    ImGui::PushStyleColor(ImGuiCol_Text, sel ? kV4Blue : kV4Text);
                    if (ImGui::Selectable(lbl, sel)) {
                        i2c.selectedIdx = i; i2c.manualAddr = addr;
                    }
                    ImGui::PopStyleColor();
                }
            }
            ImGui::EndListBox();
        }
        if (!i2c.detectedAddrs.empty()) {
            ImGui::TextColored(kV4Overlay1, "Target TX: 0x%02X", i2c.manualAddr);
        }
    }

    // ---- Live PROX Table (collapsible) ----
    bool proxOpen = ImGui::CollapsingHeader("Live PROX Table", ImGuiTreeNodeFlags_DefaultOpen);
    if (proxOpen) {
        ImGui::Checkbox("Aktif##pg", &i2c.proxGridActive);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputScalar("Poll (ms)##pg", ImGuiDataType_U32, &i2c.proxGridPollMs);
        i2c.proxGridPollMs = (uint32_t)ClampInt((int)i2c.proxGridPollMs, 50, 5000);

        if (!i2c.connected) {
            ImGui::TextColored(kV4Peach, "Connect I2C untuk melihat PROX live.");
        } else {
            ImGuiTableFlags tfl = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                                | ImGuiTableFlags_SizingFixedFit;
            if (ImGui::BeginTable("##pgt", st.cols + 1, tfl)) {
                for (int y = st.rows - 1; y >= 0; --y) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextColored(kV4Overlay1, "Y%d", y);
                    for (int x = 0; x < st.cols; ++x) {
                        ImGui::TableSetColumnIndex(x + 1);
                        int idx = y * st.cols + x, addr = 0;
                        XYToAddr(x, y, st.cols, st.rows, addr);
                        bool valid = (idx < (int)i2c.proxGridValid.size() && i2c.proxGridValid[idx]);
                        uint8_t val = valid ? i2c.proxGridVal[idx] : 0;

                        // Color cell red if prox > 0
                        if (valid && val > 0)
                            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                                IM_COL32(243, 139, 168, 80));

                        if (valid) ImGui::TextColored(val > 0 ? kV4Red : kV4Text, "%u", (unsigned)val);
                        else       ImGui::TextColored(kV4Overlay0, "--");

                        if (ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            ImGui::TextColored(kV4Lavender, "addr=0x%02X  (%d,%d)", addr, x, y);
                            if (valid) ImGui::TextColored(val > 0 ? kV4Red : kV4Green,
                                                          "prox=%u  %s", (unsigned)val,
                                                          val > 0 ? "<- OBJEK TERDETEKSI" : "");
                            else ImGui::TextColored(kV4Overlay0, "prox=N/A");
                            ImGui::EndTooltip();
                        }
                    }
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay0, "X->");
                for (int x = 0; x < st.cols; ++x) {
                    ImGui::TableSetColumnIndex(x + 1);
                    ImGui::TextColored(kV4Overlay1, "X%d", x);
                }
                ImGui::EndTable();
            }
        }
    }

    // ---- Sensing Config (debounce + delay in one section) ----
    ImGui::SeparatorText("Sensing Config");
    {
        ImGui::SetNextItemWidth(100);
        int confirmVal = (int)exe.proxConfirmNeeded;
        if (ImGui::InputInt("Konfirmasi##pc", &confirmVal))
            exe.proxConfirmNeeded = (uint8_t)ClampInt(confirmVal, 1, 10);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Jumlah pembacaan non-zero berurutan\nsebelum sensor prox dianggap terdeteksi \nuntuk mengatasi noise i2c dari motor.\n1=off, 3-5=direkomendasikan.");

        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Delay Next Command (ms)##dp", ImGuiDataType_U32, &exe.afterProxDelayMs);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Delay setelah sensor prox terdeteksi\nsebelum perintah berikutnya dikirim.");
    }

    // ---- Execute Section with prerequisite checklist ----
    ImGui::SeparatorText("Execute");
    {
        bool dijkReady  = st.path.found && !st.path.cells.empty();
        bool i2cReady   = i2c.connected;
        bool rangeReady = addrRangeOk;
        bool canExec    = dijkReady && i2cReady && rangeReady && !exe.running;

        // Checklist
        auto DrawCheck = [](bool ok, const char* label, const char* hint = nullptr) {
            ImGui::TextColored(ok ? kV4Green : kV4Red, ok ? "[OK]" : "[!!]");
            ImGui::SameLine();
            ImGui::TextColored(ok ? kV4Text : kV4Overlay1, "%s", label);
            if (hint && ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", hint);
        };

        DrawCheck(dijkReady,  "Dijkstra",
                  "Jalankan simulasi Dijkstra terlebih dahulu\nagar path tersedia untuk dieksekusi.");
        if (dijkReady) {
            ImGui::SameLine();
            ImGui::TextColored(kV4Overlay0, "(%d node, %d belokan)",
                               st.path.lastPathNodes, st.path.lastTurnCount);
        }
        DrawCheck(i2cReady,   "I2C Terhubung",
                  "Tekan tombol Connect di atas\nuntuk membuka koneksi I2C.");
        if (!rangeReady)
            DrawCheck(rangeReady, "Addr dalam range",
                      "Terlalu banyak node: addr melebihi 0x77.\nKurangi jumlah kolom/baris grid.");

        ImGui::Spacing();

        // Execute button (green when ready, gray when not)
        if (canExec) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Green.x*0.4f, kV4Green.y*0.4f, kV4Green.z*0.4f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Green.x*0.6f, kV4Green.y*0.6f, kV4Green.z*0.6f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Green);
            ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.05f,0.05f,0.05f,1.0f));
        }
        bool doExec = ImGui::Button("  Execute Simulation  ", ImVec2(-1, 0));
        if (canExec) ImGui::PopStyleColor(4);

        if (!canExec && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::BeginTooltip();
            if (!dijkReady)  ImGui::TextColored(kV4Red,    "[!!] Jalankan Dijkstra dulu!");
            if (!i2cReady)   ImGui::TextColored(kV4Red,    "[!!] Connect I2C dulu!");
            if (!rangeReady) ImGui::TextColored(kV4Yellow, "[!!] Addr range melebihi 0x77!");
            ImGui::EndTooltip();
        }

        if (doExec && canExec) {
            ResetI2CExecState(st);
            std::string err;
            if (!i2c.connected) {
                if (!I2C_Open(i2c.bus, i2c.fd, err)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] Gagal membuka I2C: %s\n",
                                   GetTimestamp().c_str(), err.c_str());
                    goto exec_done;
                }
                i2c.connected = true;
            }

            for (int addr = sF; addr <= sL; ++addr)
                (void)I2C_WriteByte(i2c.fd, addr, 9, err);
            i2c.log.AddLog("[%s][EXEC] STOP ALL dikirim ke 0x%02X..0x%02X.\n",
                           GetTimestamp().c_str(), sF, sL);

            {
                int sx = 0, sy = 0;
                if (!PortToInsideCell(st, st.portIn, sx, sy)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] Port IN tidak valid.\n",
                                   GetTimestamp().c_str());
                    goto exec_done;
                }
                int startAddr = 0;
                if (!XYToAddr(sx, sy, st.cols, st.rows, startAddr)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] Addr mapping gagal (%d,%d).\n",
                                   GetTimestamp().c_str(), sx, sy);
                    goto exec_done;
                }
                uint8_t entryCmd = CmdEnterFromPortInSide(st.portIn.side);
                if (!I2C_WriteByte(i2c.fd, startAddr, entryCmd, err)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] TX start node gagal: %s\n",
                                   GetTimestamp().c_str(), err.c_str());
                    goto exec_done;
                }

                exe.entryAddr = startAddr;
                exe.entryCmd  = entryCmd;

                if (!PrepareExecPlan_TurnGated(st)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] Persiapan plan gagal.\n",
                                   GetTimestamp().c_str());
                    ResetI2CExecState(st);
                    goto exec_done;
                }

                i2c.log.AddLog("[%s][EXEC] Plan: %d node | %d gate | delay=%u ms\n",
                               GetTimestamp().c_str(),
                               (int)exe.path.size(), (int)exe.turnGates.size(),
                               (unsigned)exe.afterProxDelayMs);

                if (!PrimeAllPathNodes_TurnGated(st)) {
                    i2c.log.AddLog("[%s][ERR][EXEC] PRIME gagal.\n", GetTimestamp().c_str());
                    ResetI2CExecState(st);
                    goto exec_done;
                }

                exe.running         = true;
                exe.phase           = I2CExecState::Phase::WaitProxStart;
                exe.entryLastPollMs = 0;
                exe.entryLastProx   = 0xFF;

                i2c.log.AddLog("[%s][EXEC] Menunggu benda di entry: addr=0x%02X (%d,%d)\n",
                               GetTimestamp().c_str(), startAddr, sx, sy);

                BuildProxWatchList_StartAndTurns(st);
                i2c.proxWatchActive = true;
            }
            exec_done:;
        }

        ImGui::Spacing();

        // STOP ALL button (always red)
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Red.x*0.4f, kV4Red.y*0.4f, kV4Red.z*0.4f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Red.x*0.6f, kV4Red.y*0.6f, kV4Red.z*0.6f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Red);
        bool doStop = ImGui::Button("  STOP ALL  ", ImVec2(-1, 0));
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Kirim cmd STOP (9) ke semua node\ndan hentikan eksekusi.");

        if (doStop) {
            std::string err;
            ResetI2CExecState(st);
            std::fill(i2c.proxGridVal.begin(),   i2c.proxGridVal.end(),   (uint8_t)0);
            std::fill(i2c.proxGridValid.begin(), i2c.proxGridValid.end(), false);
            if (!i2c.connected) {
                if (I2C_Open(i2c.bus, i2c.fd, err)) {
                    i2c.connected = true;
                }
            }
            if (i2c.connected) {
                for (int addr = sF; addr <= sL; ++addr)
                    (void)I2C_WriteByte(i2c.fd, addr, 9, err);
                i2c.log.AddLog("[%s][EXEC] STOP ALL dikirim.\n", GetTimestamp().c_str());
            }
        }

        if (exe.running) {
            ImGui::Spacing();
            uint32_t ticks = SDL_GetTicks();
            const char* dots = ((ticks / 400) % 3 == 0) ? ".  " :
                               ((ticks / 400) % 3 == 1) ? ".. " : "...";
            ImGui::TextColored(kV4Green, "Eksekusi berjalan %s", dots);
        }
    }
#endif
}

// ============================================================
//  Main window layout — resizable panels
// ============================================================
static void DrawMainWindow(GridCanvasState& st)
{
    static bool  sw      = true;
    static float s_leftW = 390.0f;
    static float s_logH  = 320.0f;
    const  float kSplitThk = 6.0f;

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(vp->Size);
    ImGui::Begin("Main Window", &sw,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImVec2 avail = ImGui::GetContentRegionAvail();

    // Clamp sizes
    s_leftW = Clamp(s_leftW, 240.0f, avail.x * 0.55f);
    s_logH  = Clamp(s_logH,  100.0f, avail.y * 0.65f);

    float topH = avail.y - s_logH - kSplitThk;
    if (topH < 120.0f) topH = 120.0f;

    // ---- Top area (left panel + canvas) ----
    ImGui::BeginChild("##TA", ImVec2(0, topH), false, ImGuiWindowFlags_NoScrollbar);
    {
        // Left panel
        ImGui::BeginChild("##LP", ImVec2(s_leftW, 0), true);
        if (ImGui::BeginTabBar("##TT", ImGuiTabBarFlags_FittingPolicyScroll)) {
            if (ImGui::BeginTabItem("SIMULATION")) { DrawLeftPanel_Simulation(st); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("I2C"))        { DrawLeftPanel_I2C(st);        ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();

        // Vertical splitter
        DrawVerticalSplitter(s_leftW, 240.0f, avail.x * 0.55f);

        // Canvas
        ImGui::BeginChild("##RP", ImVec2(0, 0), false);
        DrawCanvasChild(st);
        ImGui::EndChild();
    }
    ImGui::EndChild();

    // Horizontal splitter
    DrawHorizontalSplitter(s_logH, 100.0f, avail.y * 0.65f);

    // ---- Bottom log/result area ----
    ImGui::BeginChild("##LA", ImVec2(0, 0), true);
    if (ImGui::BeginTabBar("##BT", ImGuiTabBarFlags_FittingPolicyScroll)) {

        // ========== Tab: Result ==========
        if (ImGui::BeginTabItem("Result")) {
            // Status banner
            if (st.path.lastStartX >= 0) {
                if (st.path.lastFound) {
                    ImGui::PushStyleColor(ImGuiCol_ChildBg,
                        ImVec4(kV4Green.x*0.18f, kV4Green.y*0.18f, kV4Green.z*0.18f, 1.0f));
                    ImGui::BeginChild("##banner", ImVec2(-1, ImGui::GetFrameHeightWithSpacing() * 1.5f), true);
                    ImGui::TextColored(kV4Green, "  PATH DITEMUKAN");
                    ImGui::SameLine();
                    ImGui::TextColored(kV4Overlay1, "|  %d node  |  %.2f cm  |  %d belokan  |  %.3f ms",
                                       st.path.lastPathNodes, st.path.lastCostCm,
                                       st.path.lastTurnCount, st.path.computeMs);
                    ImGui::EndChild();
                    ImGui::PopStyleColor();
                } else {
                    ImGui::PushStyleColor(ImGuiCol_ChildBg,
                        ImVec4(kV4Red.x*0.18f, kV4Red.y*0.18f, kV4Red.z*0.18f, 1.0f));
                    ImGui::BeginChild("##banner", ImVec2(-1, ImGui::GetFrameHeightWithSpacing() * 1.5f), true);
                    ImGui::TextColored(kV4Red, "  PATH TIDAK DITEMUKAN");
                    ImGui::SameLine();
                    ImGui::TextColored(kV4Overlay1, "|  %.3f ms", st.path.computeMs);
                    ImGui::EndChild();
                    ImGui::PopStyleColor();
                }
            } else {
                ImGui::TextColored(kV4Overlay0, "Belum ada hasil Dijkstra. Tekan Run Dijkstra.");
            }

            ImGui::Spacing();

            // Stats in 2 columns
            if (st.path.lastStartX >= 0) {
                if (ImGui::BeginTable("##stats", 2, ImGuiTableFlags_SizingFixedFit)) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay1, "Start");
                    ImGui::TableSetColumnIndex(1); ImGui::TextColored(kV4Text, "(%d, %d)", st.path.lastStartX, st.path.lastStartY);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay1, "Goal");
                    ImGui::TableSetColumnIndex(1); ImGui::TextColored(kV4Text, "(%d, %d)", st.path.lastGoalX, st.path.lastGoalY);

                    if (st.path.lastFound) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay1, "Path nodes");
                        ImGui::TableSetColumnIndex(1); ImGui::TextColored(kV4Text, "%d", st.path.lastPathNodes);

                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay1, "Total cost");
                        ImGui::TableSetColumnIndex(1); ImGui::TextColored(kV4Yellow, "%.2f cm", st.path.lastCostCm);

                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextColored(kV4Overlay1, "Belokan");
                        ImGui::TableSetColumnIndex(1); ImGui::TextColored(kV4Peach, "%d", st.path.lastTurnCount);
                    }
                    ImGui::EndTable();
                }
            }

            // Mini path visualization
            if (st.path.lastFound && !st.path.lastCells.empty()) {
                ImGui::Spacing();
                ImGui::SeparatorText("Urutan Node Path");

                const int N      = (int)st.path.lastCells.size();
                const float nodeR = 9.0f;
                const float nodeSpacing = 55.0f;
                const float visH  = nodeR * 2 + 22.0f;

                ImGui::PushStyleColor(ImGuiCol_ChildBg, kV4Mantle);
                ImGui::BeginChild("##minipath", ImVec2(0, visH + 10.0f), true,
                                  ImGuiWindowFlags_HorizontalScrollbar);
                ImDrawList* dl = ImGui::GetWindowDrawList();
                ImVec2 p0 = ImGui::GetCursorScreenPos();
                float cy = p0.y + nodeR + 4.0f;
                float cx0 = p0.x + nodeR + 8.0f;

                // Connections
                for (int i = 0; i < N - 1; i++) {
                    ImVec2 a(cx0 + i * nodeSpacing, cy);
                    ImVec2 b(cx0 + (i+1) * nodeSpacing, cy);
                    dl->AddLine(a, b, kCatBlue, 2.0f);
                    // Arrowhead
                    float mid = (a.x + b.x) * 0.5f;
                    dl->AddTriangleFilled(
                        ImVec2(mid + 4, cy), ImVec2(mid - 4, cy - 4), ImVec2(mid - 4, cy + 4),
                        kCatBlue);
                }

                // Nodes
                for (int i = 0; i < N; i++) {
                    int id = st.path.lastCells[i];
                    int nx = id % st.cols, ny = id / st.cols;
                    ImVec2 c(cx0 + i * nodeSpacing, cy);

                    ImU32 fc = (i == 0)   ? kCatGreen :
                               (i == N-1) ? kCatSky   : kCatMauve;
                    dl->AddCircleFilled(c, nodeR, fc);
                    dl->AddCircle(c, nodeR, kCatSurf2, 0, 1.5f);

                    int addr = 0; XYToAddr(nx, ny, st.cols, st.rows, addr);
                    char lbl[16]; std::snprintf(lbl, sizeof(lbl), "%d,%d", nx, ny);
                    ImVec2 ts = ImGui::CalcTextSize(lbl);
                    dl->AddText(ImVec2(c.x - ts.x*0.5f, c.y + nodeR + 2.0f), kCatText, lbl);
                }

                ImGui::Dummy(ImVec2(cx0 - p0.x + (N - 1) * nodeSpacing + nodeR + 8.0f, visH));
                ImGui::EndChild();
                ImGui::PopStyleColor();

                // Node detail table
                ImGui::Spacing();
                ImGui::BeginChild("##pathcells",
                    ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 4.0f),
                    true, ImGuiWindowFlags_HorizontalScrollbar);
                for (int i = 0; i < N; i++) {
                    int id = st.path.lastCells[i];
                    int nx = id % st.cols, ny = id / st.cols;
                    int addr = 0; XYToAddr(nx, ny, st.cols, st.rows, addr);
                    ImVec4 col = (i == 0) ? kV4Green : (i == N-1) ? kV4Sky : kV4Text;
                    ImGui::TextColored(col, "[%2d] (%d,%d)  addr=0x%02X%s",
                                       i, nx, ny, addr,
                                       (i + 1 < N) ? "  ->" : "  (GOAL)");
                }
                ImGui::EndChild();

                // Quick action buttons
                ImGui::Spacing();
                if (ImGui::SmallButton("Run Dijkstra Ulang")) RunSimulationDijkstra(st);
                ImGui::SameLine();
                if (ImGui::SmallButton("Reset Path")) {
                    ClearPathResult(st);
                    st.i2c.log.AddLog("[%s][INFO] Path direset.\n", GetTimestamp().c_str());
                }
            }

            ImGui::EndTabItem();
        }

        // ========== Tab: Execution Log ==========
        if (ImGui::BeginTabItem("Execution Log")) {
            // Toolbar
            if (ImGui::SmallButton("Clear")) st.i2c.log.Clear();
            ImGui::SameLine();
            ImGui::Checkbox("Auto-scroll", &st.i2c.log.AutoScroll);
            ImGui::SameLine();
            ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
            ImGui::SameLine();
            if (st.i2c.connected)
                ImGui::TextColored(kV4Green, "I2C: ON");
            else
                ImGui::TextColored(kV4Red, "I2C: OFF");

            // Filter bar
            st.i2c.log.DrawFilterBar();

            // Log region (leave room for TX bar)
            float txBarH = ImGui::GetFrameHeightWithSpacing() * 2.8f;
            st.i2c.log.DrawScrollingRegion("##ILS", ImVec2(0, -txBarH));

            // ---- Manual TX ----
            ImGui::Separator();
            ImGui::TextColored(kV4Overlay1, "Manual TX");
            ImGui::SameLine();
            ImGui::TextColored(kV4Overlay0, "-> addr: 0x%02X", st.i2c.manualAddr);

            // Command dropdown
            static const char* kCmdLabels[] = {
                "1 - MAJU", "2 - KANAN_ATAS", "3 - KANAN", "4 - KANAN_BAWAH",
                "5 - MUNDUR", "6 - KIRI_BAWAH", "7 - KIRI", "8 - KIRI_ATAS",
                "9 - STOP", nullptr
            };
            static const char* kCmdValues[] = {
                "MAJU", "KANAN_ATAS", "KANAN", "KANAN_BAWAH",
                "MUNDUR", "KIRI_BAWAH", "KIRI", "KIRI_ATAS", "STOP"
            };
            static int s_selCmd = -1;

            ImGui::SetNextItemWidth(160);
            if (ImGui::BeginCombo("##cmdsel",
                s_selCmd < 0 ? "Pilih cmd..." : kCmdLabels[s_selCmd])) {
                for (int i = 0; kCmdLabels[i] != nullptr; i++) {
                    bool sel = (s_selCmd == i);
                    if (ImGui::Selectable(kCmdLabels[i], sel)) {
                        s_selCmd = i;
                        std::strncpy(st.i2c.txBuf, kCmdValues[i], sizeof(st.i2c.txBuf) - 1);
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();

            // Text input (Enter to send)
            ImGui::SetNextItemWidth(-80.0f);
            bool doSend = ImGui::InputText("##tx", st.i2c.txBuf, sizeof(st.i2c.txBuf),
                                           ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();

            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(kV4Blue.x*0.4f, kV4Blue.y*0.4f, kV4Blue.z*0.4f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(kV4Blue.x*0.6f, kV4Blue.y*0.6f, kV4Blue.z*0.6f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kV4Blue);
            doSend |= ImGui::Button("Send");
            ImGui::PopStyleColor(3);

            if (doSend) {
#if !defined(__linux__)
                st.i2c.log.AddLog("[%s][ERR] I2C send hanya tersedia di Linux.\n",
                                  GetTimestamp().c_str());
#else
                uint8_t cmd = 0;
                if (!ParseI2CCmd(st.i2c.txBuf, cmd)) {
                    st.i2c.log.AddLog("[%s][ERR] Format tidak valid: \"%s\"\n",
                                      GetTimestamp().c_str(), st.i2c.txBuf);
                } else {
                    std::string err;
                    if (!st.i2c.connected) {
                        if (I2C_Open(st.i2c.bus, st.i2c.fd, err)) {
                            st.i2c.connected = true;
                        } else {
                            st.i2c.log.AddLog("[%s][ERR][I2C] Gagal membuka: %s\n",
                                              GetTimestamp().c_str(), err.c_str());
                        }
                    }
                    if (st.i2c.connected) {
                        if (I2C_WriteByte(st.i2c.fd, st.i2c.manualAddr, cmd, err))
                            st.i2c.log.AddLog("[%s][TX] addr=0x%02X cmd=%d (%s)\n",
                                              GetTimestamp().c_str(),
                                              st.i2c.manualAddr, (int)cmd, st.i2c.txBuf);
                        else
                            st.i2c.log.AddLog("[%s][ERR][TX] addr=0x%02X cmd=%d: %s\n",
                                              GetTimestamp().c_str(),
                                              st.i2c.manualAddr, (int)cmd, err.c_str());
                    }
                }
                st.i2c.txBuf[0] = '\0';
                s_selCmd = -1;
                ImGui::SetKeyboardFocusHere(-1);
#endif
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
    ImGui::End();
}

// ============================================================
//  TickI2CProxMonitor_ReadOnly
// ============================================================
static void TickI2CProxMonitor_ReadOnly(GridCanvasState& st)
{
#if !defined(__linux__)
    (void)st; return;
#else
    I2CState& i2c = st.i2c;
    if (!i2c.proxWatchActive || i2c.proxWatchAddrs.empty()) return;

    const uint32_t now = (uint32_t)SDL_GetTicks();
    if (i2c.proxLastPollMs != 0 && (now - i2c.proxLastPollMs) < i2c.proxPollMs) return;
    i2c.proxLastPollMs = now;

    if (!i2c.connected) {
        std::string err;
        if (!I2C_Open(i2c.bus, i2c.fd, err)) {
            i2c.log.AddLog("[%s][ERR][PROX] Koneksi I2C hilang: %s\n",
                           GetTimestamp().c_str(), err.c_str());
            return;
        }
        i2c.connected = true;
    }

    for (size_t k = 0; k < i2c.proxWatchAddrs.size(); ++k) {
        int     addr = i2c.proxWatchAddrs[k];
        int     pi   = i2c.proxWatchPathIdx[k];
        uint8_t v    = 0; std::string err;
        if (!I2C_ReadByte(i2c.fd, addr, v, err)) {
            i2c.log.AddLog("[%s][ERR][PROX] Baca gagal addr=0x%02X (node %d): %s\n",
                           GetTimestamp().c_str(), addr, pi, err.c_str());
            continue;
        }
        int x = 0, y = 0; AddrToXY(addr, st.cols, st.rows, x, y);
        uint8_t& last = i2c.proxLastVal[k];
        if (last == 0xFF)
            i2c.log.AddLog("[%s][PROX] Init addr=0x%02X (node %d, %d,%d) prox=%u\n",
                           GetTimestamp().c_str(), addr, pi, x, y, (unsigned)v);
        else if (v != last)
            i2c.log.AddLog("[%s][PROX] Perubahan addr=0x%02X (node %d, %d,%d): %u -> %u\n",
                           GetTimestamp().c_str(), addr, pi, x, y, (unsigned)last, (unsigned)v);
        last = v;
    }
#endif
}

// ============================================================
//  TickI2CExecution — state machine (unchanged logic)
// ============================================================
static void TickI2CExecution(GridCanvasState& st)
{
#if !defined(__linux__)
    (void)st; return;
#else
    I2CState&     i2c = st.i2c;
    I2CExecState& e   = st.i2c.exec;

    if (!e.running) return;
    if (!st.path.found || st.path.cells.empty()) {
        i2c.log.AddLog("[%s][ERR][EXEC] Dibatalkan: tidak ada path aktif.\n",
                       GetTimestamp().c_str());
        ResetI2CExecState(st); return;
    }
    if (!i2c.connected || i2c.fd < 0) return;

    const uint32_t now = (uint32_t)SDL_GetTicks();
    const int totalNodes = (int)e.path.size();

    if (e.phase == I2CExecState::Phase::WaitShutdown) {
        if (!i2c.connected || i2c.fd < 0) { ResetI2CExecState(st); return; }
        uint32_t elapsed   = now - e.shutdownStartMs;
        uint32_t remaining = (elapsed >= I2CExecState::kShutdownDelayMs)
                             ? 0 : (I2CExecState::kShutdownDelayMs - elapsed);
        static uint32_t lastCountdownLog = 0;
        if (remaining > 0 && (now - lastCountdownLog) >= 1000) {
            lastCountdownLog = now;
            i2c.log.AddLog("[%s][EXEC] Shutdown dalam %.1f detik...\n",
                           GetTimestamp().c_str(), remaining / 1000.0f);
        }
        if (remaining == 0) {
            const int sF = kI2CAddrBase, sL = kI2CAddrBase + st.cols * st.rows - 1;
            std::string err;
            for (int addr = sF; addr <= sL; ++addr)
                I2C_WriteByte(i2c.fd, addr, 9, err);
            std::fill(i2c.proxGridVal.begin(),   i2c.proxGridVal.end(),   (uint8_t)0);
            std::fill(i2c.proxGridValid.begin(), i2c.proxGridValid.end(), false);
            i2c.log.AddLog("[%s][EXEC] Selesai. STOP ALL dikirim.\n", GetTimestamp().c_str());
            ResetI2CExecState(st);
        }
        return;
    }

    auto W = [&](int addr, uint8_t cmd) -> bool {
        std::string err;
        if (!I2C_WriteByte(i2c.fd, addr, cmd, err)) {
            int x = 0, y = 0; AddrToXY(addr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][ERR][EXEC] TX gagal addr=0x%02X (%d,%d) cmd=%u: %s\n",
                           GetTimestamp().c_str(), addr, x, y, (unsigned)cmd, err.c_str());
            return false;
        }
        return true;
    };

    switch (e.phase)
    {
    case I2CExecState::Phase::WaitProxStart:
    {
        if (e.entryLastPollMs != 0 && (now - e.entryLastPollMs) < e.entryPollMs) return;
        e.entryLastPollMs = now;
        uint8_t prox = 0; std::string err;
        if (!I2C_ReadByte(i2c.fd, e.entryAddr, prox, err)) { e.entryConfirmCount = 0; return; }
        if (e.entryLastProx == 0xFF || prox != e.entryLastProx) {
            int x = 0, y = 0; AddrToXY(e.entryAddr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Entry addr=0x%02X (%d,%d) prox=%u%s\n",
                           GetTimestamp().c_str(), e.entryAddr, x, y, (unsigned)prox,
                           prox > 0 ? " <- menunggu konfirmasi..." : "");
            e.entryLastProx = prox;
        }
        if (prox == 0) { e.entryConfirmCount = 0; return; }
        e.entryConfirmCount++;
        if (e.entryConfirmCount < e.proxConfirmNeeded) return;
        e.entryConfirmCount = 0;
        {
            int x = 0, y = 0; AddrToXY(e.entryAddr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Entry TERKONFIRMASI (%dx). Delay %u ms...\n",
                           GetTimestamp().c_str(), (int)e.proxConfirmNeeded, (unsigned)e.afterProxDelayMs);
        }
        e.phase        = I2CExecState::Phase::DelayAfterProx;
        e.phaseStartMs = now;
        return;
    }

    case I2CExecState::Phase::DelayAfterProx:
    {
        if ((now - e.phaseStartMs) < e.afterProxDelayMs) return;
        const int N = (int)e.path.size();
        if (N <= 0) { ResetI2CExecState(st); return; }
        if (N == 1) {
            if (!W(e.addr[0], e.outCmd[0])) { ResetI2CExecState(st); return; }
            int x = 0, y = 0; AddrToXY(e.addr[0], st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Single-node eject addr=0x%02X (%d,%d). Shutdown...\n",
                           GetTimestamp().c_str(), e.addr[0], x, y);
            e.phase = I2CExecState::Phase::WaitShutdown; e.shutdownStartMs = now; return;
        }
        if (!W(e.addr[0], e.outCmd[0])) { ResetI2CExecState(st); return; }
        {
            int x = 0, y = 0; AddrToXY(e.addr[0], st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Node 0/%d addr=0x%02X (%d,%d): outCmd=%u.\n",
                           GetTimestamp().c_str(), N - 1, e.addr[0], x, y, (unsigned)e.outCmd[0]);
        }
        SendDiagonalAssistForPathIdx(st, 0, "START", false);
        e.phase = I2CExecState::Phase::WaitProxTurn;
        return;
    }

    case I2CExecState::Phase::WaitProxTurn:
    {
        if (e.turnActive >= (int)e.turnGates.size()) {
            i2c.log.AddLog("[%s][EXEC] Semua gate selesai. Shutdown...\n", GetTimestamp().c_str());
            e.phase = I2CExecState::Phase::WaitShutdown; e.shutdownStartMs = now; return;
        }
        I2CExecState::TurnGate& g = e.turnGates[e.turnActive];
        if (g.done) { e.turnActive++; return; }
        if (g.lastPollMs != 0 && (now - g.lastPollMs) < e.turnPollMs) return;
        g.lastPollMs = now;
        uint8_t prox = 0; std::string err;
        if (!I2C_ReadByte(i2c.fd, g.addr, prox, err)) { g.confirmCount = 0; return; }
        if (g.lastProx == 0xFF || prox != g.lastProx) {
            int x = 0, y = 0; AddrToXY(g.addr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Gate %d/%d addr=0x%02X (%d,%d) prox=%u%s\n",
                           GetTimestamp().c_str(), e.turnActive+1, (int)e.turnGates.size(),
                           g.addr, x, y, (unsigned)prox, prox > 0 ? " <- menunggu..." : "");
            g.lastProx = prox;
        }
        if (prox == 0) { g.confirmCount = 0; return; }
        g.confirmCount++;
        if (g.confirmCount < e.proxConfirmNeeded) return;
        g.confirmCount = 0;
        g.detectMs = now;
        e.phase = I2CExecState::Phase::DelayAfterTurn;
        {
            int x = 0, y = 0; AddrToXY(g.addr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Gate %d/%d TERKONFIRMASI. Delay %u ms...\n",
                           GetTimestamp().c_str(), e.turnActive+1,
                           (int)e.turnGates.size(), (unsigned)e.afterProxDelayMs);
        }
        return;
    }

    case I2CExecState::Phase::DelayAfterTurn:
    {
        if (e.turnActive >= (int)e.turnGates.size()) { ResetI2CExecState(st); return; }
        I2CExecState::TurnGate& g = e.turnGates[e.turnActive];
        if ((now - g.detectMs) < e.afterProxDelayMs) return;
        if (!W(g.addr, g.outCmd)) { ResetI2CExecState(st); return; }
        {
            int x = 0, y = 0; AddrToXY(g.addr, st.cols, st.rows, x, y);
            i2c.log.AddLog("[%s][EXEC] Gate %d/%d: addr=0x%02X (%d,%d) %u->%u | node %d/%d\n",
                           GetTimestamp().c_str(), e.turnActive+1, (int)e.turnGates.size(),
                           g.addr, x, y, (unsigned)g.inCmd, (unsigned)g.outCmd,
                           g.pathIdx, totalNodes - 1);
        }
        SendDiagonalAssistForPathIdx(st, g.pathIdx, "TURN", false);
        g.done = true; e.turnActive++;
        if (e.turnActive < (int)e.turnGates.size())
            e.turnGates[e.turnActive].confirmCount = 0;
        e.phase = I2CExecState::Phase::WaitProxTurn;
        return;
    }

    default:
        ResetI2CExecState(st);
        return;
    }
#endif
}

// ============================================================
//  SDL + GL init
// ============================================================
static bool InitSDLAndGL(SDL_Window** outWindow, SDL_GLContext* outContext)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::printf("SDL_Init: %s\n", SDL_GetError()); return false;
    }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
#if defined(USE_GLES2)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
#endif
    SDL_Window* w = SDL_CreateWindow(
        "4-Wheeled Omnidirectional Conveyor Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 800,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);
    if (!w) { std::printf("SDL_CreateWindow: %s\n", SDL_GetError()); return false; }
    SDL_MaximizeWindow(w);
    SDL_GLContext gl = SDL_GL_CreateContext(w);
    if (!gl) {
        std::printf("SDL_GL_CreateContext: %s\n", SDL_GetError());
        SDL_DestroyWindow(w); return false;
    }
    SDL_GL_SetSwapInterval(0);
    *outWindow = w; *outContext = gl;
    return true;
}

// ============================================================
//  main
// ============================================================
int main(int, char**)
{
    SDL_Window* window = nullptr; SDL_GLContext gl = nullptr;
    if (!InitSDLAndGL(&window, &gl)) return 1;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // disable imgui.ini

    // ---- Font loading (try system TTF, fallback to scaled default) ----
    bool fontLoaded = false;
    const char* fontPaths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        nullptr
    };
    for (int i = 0; fontPaths[i] != nullptr && !fontLoaded; i++) {
        if (io.Fonts->AddFontFromFileTTF(fontPaths[i], 15.0f)) {
            fontLoaded = true;
        }
    }
    if (!fontLoaded) {
        // Scale up the built-in bitmap font
        io.FontGlobalScale = 1.3f;
    }

    // ---- Apply Catppuccin Mocha theme ----
    ApplyCatppuccinTheme();

    ImGui_ImplSDL2_InitForOpenGL(window, gl);
#if defined(USE_GLES2)
    ImGui_ImplOpenGL3_Init("#version 100");
#else
    ImGui_ImplOpenGL2_Init();
#endif

    bool done = false;
    GridCanvasState st;

    static constexpr uint32_t kTargetFPS     = 60;
    static constexpr uint32_t kFrameBudgetMs = 1000u / kTargetFPS;
    uint32_t frameStart = SDL_GetTicks();

    while (!done) {
        frameStart = SDL_GetTicks();

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            if (e.type == SDL_QUIT) done = true;
            if (e.type == SDL_WINDOWEVENT &&
                e.window.event == SDL_WINDOWEVENT_CLOSE &&
                e.window.windowID == SDL_GetWindowID(window)) done = true;
        }

#if defined(USE_GLES2)
        ImGui_ImplOpenGL3_NewFrame();
#else
        ImGui_ImplOpenGL2_NewFrame();
#endif
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        DrawMainWindow(st);
        TickI2CExecution(st);
        TickI2CProxMonitor_ReadOnly(st);
        TickI2CProxGrid_AllNodes(st);

        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        // Catppuccin Mocha base color as clear
        glClearColor(0.118f, 0.118f, 0.180f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

#if defined(USE_GLES2)
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#else
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#endif
        SDL_GL_SwapWindow(window);

        uint32_t frameMs = SDL_GetTicks() - frameStart;
        if (frameMs < kFrameBudgetMs)
            SDL_Delay(kFrameBudgetMs - frameMs);
    }

#if defined(USE_GLES2)
    ImGui_ImplOpenGL3_Shutdown();
#else
    ImGui_ImplOpenGL2_Shutdown();
#endif
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
