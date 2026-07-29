#ifndef SYS_INFO_H
#define SYS_INFO_H

#include <cstdint>

namespace SysInfo {
    struct RamUsage {
        uint64_t totalBytes;
        uint64_t usedBytes;
        double usagePercent;
    };

    RamUsage GetRamUsage();
    
    void InitCpuUsage();
    
    double GetCpuUsage(); 
}

#if defined(_WIN32)
    #include <windows.h>
    
    static uint64_t prevTotalTicks = 0;
    static uint64_t prevIdleTicks = 0;

    static uint64_t FileTimeToUInt64(const FILETIME& ft) {
        return (static_cast<uint64_t>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
    }

    SysInfo::RamUsage SysInfo::GetRamUsage() {
        MEMORYSTATUSEX memInfo;
        memInfo.dwLength = sizeof(MEMORYSTATUSEX);
        GlobalMemoryStatusEx(&memInfo);
        
        SysInfo::RamUsage usage;
        usage.totalBytes = memInfo.ullTotalPhys;
        usage.usedBytes = memInfo.ullTotalPhys - memInfo.ullAvailPhys;
        usage.usagePercent = (static_cast<double>(usage.usedBytes) / usage.totalBytes) * 100.0;
        return usage;
    }

    void SysInfo::InitCpuUsage() {
        FILETIME idleTime, kernelTime, userTime;
        if (GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
            prevIdleTicks = FileTimeToUInt64(idleTime);
            prevTotalTicks = FileTimeToUInt64(kernelTime) + FileTimeToUInt64(userTime);
        }
    }

    double SysInfo::GetCpuUsage() {
        FILETIME idleTime, kernelTime, userTime;
        if (!GetSystemTimes(&idleTime, &kernelTime, &userTime)) {
            return 0.0;
        }

        uint64_t curIdle = FileTimeToUInt64(idleTime);
        uint64_t curTotal = FileTimeToUInt64(kernelTime) + FileTimeToUInt64(userTime);

        uint64_t diffIdle = curIdle - prevIdleTicks;
        uint64_t diffTotal = curTotal - prevTotalTicks;

        prevIdleTicks = curIdle;
        prevTotalTicks = curTotal;

        if (diffTotal == 0) return 0.0;
        return (static_cast<double>(diffTotal - diffIdle) / diffTotal) * 100.0;
    }

#elif defined(__linux__)
    #include <sys/sysinfo.h>
    #include <fstream>
    #include <string>
    #include <sstream>

    static uint64_t prevTotalTicks = 0;
    static uint64_t prevIdleTicks = 0;

    struct CpuJiffies {
        uint64_t idle;
        uint64_t total;
    };

    static CpuJiffies ReadProcStat() {
        std::ifstream file("/proc/stat");
        std::string line;
        if (std::getline(file, line)) {
            if (line.compare(0, 4, "cpu ") == 0) {
                std::istringstream iss(line.substr(4));
                uint64_t user = 0, nice = 0, system = 0, idle = 0, iowait = 0, 
                         irq = 0, softirq = 0, steal = 0;
                
                iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

                uint64_t idleTime = idle + iowait;
                uint64_t nonIdleTime = user + nice + system + irq + softirq + steal;
                
                return { idleTime, idleTime + nonIdleTime };
            }
        }
        return {0, 0};
    }

    SysInfo::RamUsage SysInfo::GetRamUsage() {
        std::ifstream meminfo("/proc/meminfo");
        std::string line;
        uint64_t memTotal = 0, memAvailable = 0;
        
        while(std::getline(meminfo, line)) {
            if (line.compare(0, 9, "MemTotal:") == 0) {
                sscanf(line.c_str(), "MemTotal: %lu kB", &memTotal);
            } else if (line.compare(0, 13, "MemAvailable:") == 0) {
                sscanf(line.c_str(), "MemAvailable: %lu kB", &memAvailable);
            }
        }
        
        memTotal *= 1024;
        memAvailable *= 1024;
        uint64_t memUsed = memTotal - memAvailable;
        
        SysInfo::RamUsage usage;
        usage.totalBytes = memTotal;
        usage.usedBytes = memUsed;
        usage.usagePercent = memTotal > 0 ? (static_cast<double>(usage.usedBytes) / usage.totalBytes) * 100.0 : 0.0;
        return usage;
    }

    void SysInfo::InitCpuUsage() {
        CpuJiffies jiffies = ReadProcStat();
        prevIdleTicks = jiffies.idle;
        prevTotalTicks = jiffies.total;
    }

    double SysInfo::GetCpuUsage() {
        CpuJiffies curJiffies = ReadProcStat();
        
        uint64_t diffIdle = curJiffies.idle - prevIdleTicks;
        uint64_t diffTotal = curJiffies.total - prevTotalTicks;

        prevIdleTicks = curJiffies.idle;
        prevTotalTicks = curJiffies.total;

        if (diffTotal == 0) return 0.0;
        return (static_cast<double>(diffTotal - diffIdle) / diffTotal) * 100.0;
    }

#else
    #error "Unsupported Platform!"
#endif

#endif // SYS_INFO_H