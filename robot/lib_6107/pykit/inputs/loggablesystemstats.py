from hal import (
    getBrownedOut,
    getComments,
    getCommsDisableCount,
    getFPGAButton,
    getFPGARevision,
    getFPGAVersion,
    getRSLState,
    getSerialNumber,
    getSystemActive,
    getSystemTimeValid,
    getTeamNumber,
)
from ntcore import NetworkTableInstance

from lib_6107.pykit.logtable import LogTable


class LoggedSystemStats:
    lastNTRemoteIds: set[str] = set()
    save_pass: int = 0

    @classmethod
    def saveToTable(cls, table: LogTable):
        # Limit how and when to save off statistics here since this can be expensive
        # These are only called the first time
        if cls.save_pass == 0:
            # for some reason these return tuples of length 2, take the first element
            table.put("FPGAVersion", getFPGAVersion()[0])
            table.put("FPGARevision", getFPGARevision()[0])
            table.put("SerialNumber", getSerialNumber())
            table.put("Comments", getComments())
            table.put("TeamNumber", getTeamNumber())
            table.put("FPGAButton", getFPGAButton()[0])

        # These at about once every 4 seconds
        if cls.save_pass % 199 == 0:
            table.put("SystemActive", getSystemActive()[0])
            table.put("BrownedOut", getBrownedOut()[0])
            table.put("CommsDisabledCount", getCommsDisableCount()[0])
            table.put("RSLState", getRSLState()[0])
            table.put("SystemTimeValid", getSystemTimeValid()[0])

        # These as well at about 10 seconds
        if cls.save_pass % 499 == 0:
            ntClientsTable = table.getSubTable("NTClients")

            ntConnections = NetworkTableInstance.getDefault().getConnections()

            ntRemoteIds = set()
            for connection in ntConnections:
                if connection.remote_id in LoggedSystemStats.lastNTRemoteIds:
                    LoggedSystemStats.lastNTRemoteIds.remove(connection.remote_id)
                ntRemoteIds.add(connection.remote_id)

                ntClientTable = ntClientsTable.getSubTable(connection.remote_id)

                ntClientTable.put("Connected", True)
                ntClientTable.put("IPAddress", connection.remote_ip)
                ntClientTable.put("RemotePort", connection.remote_port)
                ntClientTable.put("ProtocolVersion", connection.protocol_version)

            # Mark disconnected clients
            for remoteId in LoggedSystemStats.lastNTRemoteIds:
                ntClientTable = ntClientsTable.getSubTable(remoteId)
                ntClientTable.put("Connected", False)

            LoggedSystemStats.lastNTRemoteIds = ntRemoteIds
        cls.save_pass += 1
