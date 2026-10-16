import asyncio
from mavsdk_grpc import System


async def main():
    drone = System()

    print("Dang ket noi den PX4 SITL...")

    # Cổng UDP phổ biến của PX4 SITL dành cho MAVSDK.
    # Nếu cấu hình hiện tại khác, hãy điều chỉnh địa chỉ này.
    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    print("Dang cho PX4...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Da ket noi thanh cong voi PX4!")
            break

    print("Dang cho du lieu telemetry...")

    async for position in drone.telemetry.position():
        print(
            f"Latitude: {position.latitude_deg:.7f}, "
            f"Longitude: {position.longitude_deg:.7f}, "
            f"Altitude: {position.relative_altitude_m:.2f} m"
        )
        break

    print("Kiem tra ket noi hoan tat.")


if __name__ == "__main__":
    asyncio.run(main())
