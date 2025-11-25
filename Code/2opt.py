import asyncio
import json
import math
import time
from mavsdk import System


class DroneDelivery2Opt:
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
            print(f"🏠 Home: Lat={self.home_position['lat']:.6f}, "
                  f"Lon={self.home_position['lon']:.6f}, "
                  f"Alt={self.home_position['alt']:.1f}m")
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
                print(f" • {point['name']}: Lat={point['lat']:.6f}, "
                      f"Lon={point['lon']:.6f}, Alt={point['alt']:.1f}m")

        except FileNotFoundError:
            print(f"❌ Không tìm thấy file: {plan_file}")
        except json.JSONDecodeError:
            print(f"❌ File không đúng định dạng JSON: {plan_file}")
        except Exception as e:
            print(f"❌ Lỗi khi đọc file: {e}")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def calculate_route_distance(self, route):
        if not route or not self.home_position:
            return float('inf')

        total = 0

        total += self.haversine_distance(
            self.home_position['lat'], self.home_position['lon'],
            route[0]['lat'], route[0]['lon'])

        for i in range(len(route) - 1):
            total += self.haversine_distance(
                route[i]['lat'], route[i]['lon'],
                route[i+1]['lat'], route[i+1]['lon']
            )

        total += self.haversine_distance(
            route[-1]['lat'], route[-1]['lon'],
            self.home_position['lat'], self.home_position['lon']
        )

        return total

    def nearest_neighbor_route(self):
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
                    current_lat, current_lon, p['lat'], p['lon']
                )
            )
            route.append(nearest)
            remaining.remove(nearest)
            current_lat = nearest['lat']
            current_lon = nearest['lon']

        return route

    def two_opt_optimize(self, route):
        if len(route) < 4:
            return route

        improved = True
        best_route = route.copy()
        best_distance = self.calculate_route_distance(best_route)
        iterations = 0

        print("🔄 Đang tối ưu route bằng 2-Opt...")

        while improved:
            improved = False
            iterations += 1

            for i in range(len(best_route) - 1):
                for j in range(i + 2, len(best_route)):

                    new_route = best_route[:i+1] + best_route[i+1:j+1][::-1] + best_route[j+1:]
                    new_distance = self.calculate_route_distance(new_route)

                    if new_distance < best_distance:
                        best_route = new_route
                        best_distance = new_distance
                        improved = True

        return best_route

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

        print(f"📍 Bay đến: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m "
              f"(khoảng cách: {distance:.1f}m)")

        await self.drone.action.goto_location(lat, lon, alt, yaw)

        while True:
            async for position in self.drone.telemetry.position():
                current_distance = self.haversine_distance(
                    position.latitude_deg, position.longitude_deg, lat, lon)

                if current_distance < 0.1:
                    segment_time = time.time() - segment_start
                    print(f"✅ Đã đến vị trí (còn cách {current_distance:.1f}m) - "
                          f"Thời gian: {segment_time:.1f}s, Quãng đường: {distance:.1f}m")
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

        initial_route = self.nearest_neighbor_route()
        initial_distance = self.calculate_route_distance(initial_route)
        print(f"📏 Quãng đường ban đầu: {initial_distance:.1f}m")

        optimized_route = self.two_opt_optimize(initial_route)

        print(f"\n📋 Route giao hàng tối ưu ({len(optimized_route)} điểm):")
        estimated_distance = 0
        current_lat = self.home_position['lat']
        current_lon = self.home_position['lon']

        for i, point in enumerate(optimized_route, 1):
            distance = self.haversine_distance(
                current_lat, current_lon, point['lat'], point['lon'])
            estimated_distance += distance
            print(f" {i}. {point['name']}: {distance:.1f}m từ điểm trước")
            current_lat = point['lat']
            current_lon = point['lon']

        back_home = self.haversine_distance(
            current_lat, current_lon,
            self.home_position['lat'], self.home_position['lon'])

        estimated_distance += back_home
        print(f" Về home: {back_home:.1f}m")
        print(f"📏 Tổng quãng đường ước tính: {estimated_distance:.1f}m")

        self.start_time = time.time()
        takeoff_alt = optimized_route[0]['alt'] if optimized_route else 10

        await self.arm_and_takeoff(altitude=takeoff_alt)

        print("\n🚁 Bắt đầu giao hàng...")
        for i, point in enumerate(optimized_route, 1):
            print(f"\n--- Điểm {i}/{len(optimized_route)} ---")
            await self.fly_to_gps_location(point['lat'], point['lon'], point['alt'])
            await self.deliver_package(point['name'])

        print("\n🏠 Quay về điểm xuất phát...")
        await self.fly_to_gps_location(
            self.home_position['lat'], self.home_position['lon'],
            self.home_position['alt'] + takeoff_alt
        )

        print("🛬 Hạ cánh...")
        await self.drone.action.land()
        await asyncio.sleep(5)

        self.total_time = time.time() - self.start_time
        self.total_distance = sum(self.segment_distances)

        print("\n" + "="*70)
        print("📊 THỐNG KÊ CHUYẾN BAY - 2-OPT OPTIMIZATION")
        print("="*70)
        print(f"⏱️ Tổng thời gian bay: {self.total_time:.1f}s ({self.total_time/60:.2f} phút)")
        print(f"📏 Tổng quãng đường: {self.total_distance:.1f}m")
        print(f"🎯 Số điểm giao hàng: {len(optimized_route)}")
        print(f"⚡ Tốc độ trung bình: {self.total_distance/self.total_time:.2f} m/s")
        print("="*70)
        print("✅ Hoàn thành mission giao hàng!")


async def main():
    delivery = DroneDelivery2Opt()

    plan_file = "/home/miyamura/Desktop/demo3.plan"
    delivery.load_mission_from_qgc(plan_file)

    if delivery.delivery_points:
        print("\n" + "="*70)
        await delivery.execute_delivery_mission()
    else:
        print("\n❌ Không có điểm giao hàng. Vui lòng kiểm tra file .plan")


if __name__ == "__main__":
    asyncio.run(main())
