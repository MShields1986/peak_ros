#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include "tcp_client_boost.h"



class PeakHandler {
public:
    PeakHandler(
        const int& frequency,
        const std::string& ip_address,
        const int& port,
        const std::string& mps_file);
    ~PeakHandler();


    void                               logToConsole(const std::string& message);
    void                               errorToConsole(const std::string& message);
    void                               setReconstructionConfiguration(
                                                        const int& n_elements,
                                                        const double& element_pitch,         // mm
                                                        const double& inter_element_spacing, // mm
                                                        const double& element_width,         // mm
                                                        const double& vel_wedge,             // m/s
                                                        const double& vel_couplant,          // m/s
                                                        const double& vel_material,          // m/s
                                                        const double& wedge_angle,           // degrees
                                                        const double& wedge_depth,           // mm
                                                        const double& specimen_depth,        // mm
                                                        const double& couplant_depth);       // mm
    void                               readMpsFile();
    std::vector<std::string>           processMpsLine(const std::string& command);
    void                               setDof(const std::string& command);
    void                               setGates(const std::string& command);
    void                               setNumAScans(const std::string& command);
    void                               calcPacketLength();

    void                               connect(int digitisation_rate = 0);
    void                               sendCommand(const std::string& command);
    void                               sendReset(int digitisation_rate);
    void                               sendMpsConfiguration();
    auto                               dataOutpoutFormatReader(const std::vector<unsigned char>& packet);
    bool                               sendDataRequest();


// Output data structures: made to stay close to the LTPA DOF message
    enum DofHeaderByte {
        ascan,                         // 1A Hex
        normal_indications,            // 1C Hex
        gain_reduced_indications,      // 1D Hex
        lwl_coupling_failure,          // 1E Hex
        error                          // 0x6 and everything else
    };

    struct DofMessageHeader {
        DofHeaderByte                  header;
        int                            count;
        int                            testNo;
        int                            dof;
        int                            channel;
    };

    struct DofMessage {
        DofMessageHeader               header;
        std::vector<short int>         amps;
    };

    struct OutputFormat {
        int                            digitisation_rate;     // MHz
        int                            ascan_length;
        int                            num_a_scans;
        int                            n_elements;
        double                         element_pitch;         // mm
        double                         inter_element_spacing; // mm
        double                         element_width;         // mm
        double                         vel_wedge;             // m/s
        double                         vel_couplant;          // m/s
        double                         vel_material;          // m/s
        double                         wedge_angle;           // degrees
        double                         wedge_depth;           // mm
        double                         couplant_depth;        // mm
        double                         specimen_depth;        // mm
        std::vector<DofMessage>        ascans;
    };

private:
    // TODO: Consider using a mutex or atomic here to avoid a race condition
    OutputFormat                       ltpa_data_;
    OutputFormat*                      ltpa_data_ptr_;
public:
    const OutputFormat*                ltpa_data_ptr() const { return ltpa_data_ptr_; };

private:
    const int                                  sub_header_size_;
    const int                                  frequency_;
    const std::string                          ip_address_;
    const int                                  port_;
    BoostSocketWrappers::TcpClientBoost        ltpa_client_;
    const std::string                          mps_file_;
    std::vector<std::string>                   commands_;

public:
    int                                dof_;
    int                                gate_start_;
    int                                gate_end_;
    int                                ascan_length_;
    int                                num_a_scans_;

private:
    int                                individual_ascan_obs_length_;
    int                                packet_length_;

};