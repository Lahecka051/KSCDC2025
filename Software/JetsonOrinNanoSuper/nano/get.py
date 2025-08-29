        self.ARRIVAL_RADIUS = 2.0   #목표지점 도착 오차허용 거리

    def get_distance_metres(self,lat1, lon1, lat2, lon2):
        """두 GPS 좌표 사이의 거리를 미터 단위로 계산하는 함수 (하버사인 공식)"""
        R = 6371e3  # 지구의 반지름 (미터)
        
        # 위도, 경도를 라디안으로 변환
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance
