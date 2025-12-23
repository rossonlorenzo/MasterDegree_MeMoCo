#pragma once
#include "board.h"
#include <vector>
#include <string>

class PatternLibrary
{
public:
    static const std::vector<Pattern>& all()
    {
        static std::vector<Pattern> patterns = {

            // =========================
            // SOIC-8 (1.27mm pitch)
            // =========================
            {
                "SOIC_8",
                {
                    {
                        "IC_SOIC8",
                        {
                            // Left side pins
                            {0.0,  0.0},
                            {0.0,  1.27},
                            {0.0,  2.54},
                            {0.0,  3.81},

                            // Right side pins
                            {5.4,  0.0},
                            {5.4,  1.27},
                            {5.4,  2.54},
                            {5.4,  3.81}
                        }
                    }
                }
            },

            // =========================
            // 2-pin Connector (2.54mm pitch)
            // =========================
            {
                "CONN_2P_2.54",
                {
                    {
                        "Connector_2P",
                        {
                            {0.0, 0.0},
                            {2.54, 0.0}
                        }
                    }
                }
            },

            // =========================
            // 4-pin Connector (2x2)
            // =========================
            {
                "CONN_4P_2x2",
                {
                    {
                        "Connector_4P",
                        {
                            {0.0, 0.0}, {2.54, 0.0},
                            {0.0, 2.54}, {2.54, 2.54}
                        }
                    }
                }
            },

            // =========================
            // Mounting Hole (M3)
            // =========================
            {
                "MOUNT_M3",
                {
                    {
                        "MountingHole_M3",
                        {
                            {0.0, 0.0}
                        }
                    }
                }
            },

            // =========================
            // VIA (single)
            // =========================
            {
                "VIA_SINGLE",
                {
                    {
                        "Via",
                        {
                            {0.0, 0.0}
                        }
                    }
                }
            },

            // =========================
            // VIA cluster (2x2 grid)
            // =========================
            {
                "VIA_CLUSTER_2x2",
                {
                    {
                        "ViaArray",
                        {
                            {0.0, 0.0}, {1.0, 0.0},
                            {0.0, 1.0}, {1.0, 1.0}
                        }
                    }
                }
            },

            // =========================
            // USB-like connector (simplified)
            // =========================
            {
                "USB_CONN_SIMPLIFIED",
                {
                    {
                        "USB_Data_Pins",
                        {
                            {0.0, 0.0},
                            {0.65, 0.0},
                            {1.30, 0.0},
                            {1.95, 0.0}
                        }
                    },
                    {
                        "USB_Mounting_Pins",
                        {
                            {-1.5, -2.0},
                            { 3.5, -2.0}
                        }
                    }
                }
            }
        };

        return patterns;
    }
};
