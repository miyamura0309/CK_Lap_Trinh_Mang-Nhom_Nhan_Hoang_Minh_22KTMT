import asyncio
import json
import math
import time
from mavsdk import System


class DroneDeliveryNearestNeighbor:
    def __init__(self):
        self.drone = System()
        self.delivery_points = []
        self.home_position = None

        self.total_distance = 0
        self.total_time = 0
        self.start_time = None

        self.segment_distances = []
        self.segment_times = []

    async def connect(self):
        print("🔌 Đang kết nối với drone...")
        await self.drone.connect(system_address="udp://:14540")

        print("⏳ Đợi drone sẵn sàng...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Đã kết nối với drone!")
                break

        print("📍 Lấy vị trí home...")
        async for position in self.drone.telemetry.position():
            self.home_position = {
                'lat': position.latitude_deg,
                'lon': position.longitude_deg,
                'alt': position.absolute_altitude_m
            }
            print(
                f"🏠 Home: Lat={self.home_position['lat']:.6f}, "
                f"Lon={self.home_position['lon']:.6f}, "
                f"Alt={self.home_position['alt']:.1f}m"
            )
            break

    def load_mission_from_qgc(self, plan_file):
        print(f"📂 Đang đọc file: {plan_file}")

        try:
            with open(plan_file, 'r') as f:
                data = json.load(f)

            items = data.get('mission', {}).get('items', [])

            for i, item in enumerate(items):
                if item.get('type') == 'SimpleItem' and item.get('command') == 16:
                    params = item.get('params', [])
                    if len(params) >= 7:
                        self.delivery_points.append({
                            'name': f'Điểm {i+1}',
                            'lat': params[4],
                            'lon': params[5],
                            'alt': params[6]
                        })

            print(f"✅ Đã load {len(self.delivery_points)} điểm giao hàng:")
            for point in self.delivery_points:
                print(
                    f" • {point['name']}: "
                    f"Lat={point['lat']:.6f}, Lon={point['lon']:.6f}, "
                    f"Alt={point['alt']:.1f}m"
                )

        except FileNotFoundError:
            print(f"❌ Không tìm thấy file: {plan_file}")
        except json.JSONDecodeError:
            print(f"❌ File không đúng định dạng JSON: {plan_file}")
        except Exception as e:
            print(f"❌ Lỗi khi đọc file: {e}")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Tính khoảng cách giữa 2 điểm GPS (Haversine)"""
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)

        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def nearest_neighbor_route(self):
        """Thuật toán nearest neighbor để tối ưu route"""
        if not self.delivery_points or not self.home_position:
            return []

        route = []
        remaining = self.delivery_points.copy()

        current_lat = self.home_position['lat']
        current_lon = self.home_position['lon']

        while remaining:
            nearest = min(
                remaining,
                key=lambda p: self.haversine_distance(
                    current_lat, current_lon, p['lat'], p['lon'])
            )
            route.append(nearest)
            remaining.remove(nearest)
            current_lat = nearest['lat']
            current_lon = nearest['lon']

        return route

    async def arm_and_takeoff(self, altitude=10):
        print("🔧 Đang arm drone...")
        await self.drone.action.arm()

        print(f"🚁 Cất cánh lên độ cao {altitude}m...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        await asyncio.sleep(10)

        print("✅ Đã cất cánh")

    async def fly_to_gps_location(self, lat, lon, alt, yaw=float('nan')):
        segment_start = time.time()

        async for position in self.drone.telemetry.position():
            start_lat = position.latitude_deg
            start_lon = position.longitude_deg
            break

        distance = self.haversine_distance(start_lat, start_lon, lat, lon)
        print(
            f"📍 Bay đến: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m "
            f"(khoảng cách: {distance:.1f}m)"
        )

        await self.drone.action.goto_location(lat, lon, alt, yaw)

        while True:
            async for position in self.drone.telemetry.position():
                current_distance = self.haversine_distance(
                    position.latitude_deg, position.longitude_deg, lat, lon)

                if current_distance < 0.1:
                    segment_time = time.time() - segment_start
                    print(
                        f"✅ Đã đến (cách {current_distance:.1f}m) - "
                        f"Thời gian: {segment_time:.1f}s, "
                        f"Quãng đường: {distance:.1f}m"
                    )
                    self.segment_distances.append(distance)
                    self.segment_times.append(segment_time)
                    return

            await asyncio.sleep(1)

    async def deliver_package(self, point_name, duration=5):
        print(f"📦 Đang giao hàng tại {point_name}...")
        await asyncio.sleep(duration)
        print(f"✅ Đã giao hàng tại {point_name}")

    async def execute_delivery_mission(self):
        if not self.delivery_points:
            print("❌ Không có điểm giao hàng nào!")
            return

        await self.connect()

        print("\n🗺️ Tính toán route tối ưu...")
        optimized_route = self.nearest_neighbor_route()

        print(f"📋 Route gồm {len(optimized_route)} điểm:")

        estimated_distance = 0
        current_lat = self.home_position['lat']
        current_lon = self.home_position['lon']

        for i, point in enumerate(optimized_route, 1):
            d = self.haversine_distance(
                current_lat, current_lon, point['lat'], point['lon']
            )
            estimated_distance += d

            print(f" {i}. {point['name']} — {d:.1f}m")

            current_lat = point['lat']
            current_lon = point['lon']

        back_home = self.haversine_distance(
            current_lat, current_lon,
            self.home_position['lat'], self.home_position['lon']
        )

        estimated_distance += back_home
        print(f"🔁 Quay về home: {back_home:.1f}m")
        print(f"📏 Tổng quãng đường ước tính: {estimated_distance:.1f}m")

        self.start_time = time.time()

        takeoff_alt = optimized_route[0]['alt'] if optimized_route else 10
        await self.arm_and_takeoff(takeoff_alt)

        print("\n🚁 Bắt đầu giao hàng...")

        for i, point in enumerate(optimized_route, 1):
            print(f"\n--- Điểm {i}/{len(optimized_route)} ---")
            await self.fly_to_gps_location(point['lat'], point['lon'], point['alt'])
            await self.deliver_package(point['name'])

        print("\n🏠 Quay về home...")
        await self.fly_to_gps_location(
            self.home_position['lat'],
            self.home_position['lon'],
            self.home_position['alt'] + takeoff_alt
        )

        print("🛬 Hạ cánh...")
        await self.drone.action.land()
        await asyncio.sleep(5)

        self.total_time = time.time() - self.start_time
        self.total_distance = sum(self.segment_distances)

        print("\n" + "=" * 70)
        print("📊 THỐNG KÊ CHUYẾN BAY — NEAREST NEIGHBOR")
        print("=" * 70)
        print(f"⏱️ Thời gian bay: {self.total_time:.1f}s")
        print(f"📏 Tổng quãng đường: {self.total_distance:.1f}m")
        print(f"🎯 Số điểm giao hàng: {len(optimized_route)}")
        print(f"⚡ Tốc độ TB: {self.total_distance / self.total_time:.2f} m/s")

        print("\n📍 Chi tiết từng đoạn:")
        for i, (dist, t) in enumerate(zip(self.segment_distances, self.segment_times), 1):
            name = optimized_route[i-1]['name'] if i <= len(optimized_route) else "Home"
            print(f" Đoạn {i} ({name}): {dist:.1f}m — {t:.1f}s — {dist/t:.2f} m/s")

        print("=" * 70)
        print("✅ Hoàn thành mission giao hàng!")

async def main():
    delivery = DroneDeliveryNearestNeighbor()
    plan_file = "/home/miyamura/Desktop/demo2.plan"

    delivery.load_mission_from_qgc(plan_file)

    if delivery.delivery_points:
        print("\n" + "="*70)
        await delivery.execute_delivery_mission()
    else:
        print("❌ Không có điểm giao hàng, kiểm tra file .plan")


if __name__ == "__main__":
    asyncio.run(main())
