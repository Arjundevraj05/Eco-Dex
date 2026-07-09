'use client';
import { useEffect, useRef, useState } from 'react';
import type { LatLngTuple, Map as LeafletMap, FeatureGroup, Icon } from 'leaflet';
import 'leaflet/dist/leaflet.css';

interface MapComponentProps {
  locations: { lat: number; lng: number }[];
}

const DEFAULT_CENTER: LatLngTuple = [12.9716, 77.5946];

const MapComponent: React.FC<MapComponentProps> = ({ locations }) => {
  const mapRef = useRef<LeafletMap | null>(null);
  const markersRef = useRef<FeatureGroup | null>(null);
  const iconRef = useRef<Icon | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [ready, setReady] = useState(false);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    let cancelled = false;

    const init = async () => {
      const L = (await import('leaflet')).default;
      if (cancelled || !containerRef.current || mapRef.current) return;

      iconRef.current = new L.Icon({
        iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
        iconSize: [25, 41],
        iconAnchor: [12, 41],
        popupAnchor: [1, -34],
        shadowSize: [41, 41],
      });

      const map = L.map(containerRef.current).setView(DEFAULT_CENTER, 12);
      mapRef.current = map;

      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors',
      }).addTo(map);

      markersRef.current = L.featureGroup().addTo(map);
      setReady(true);
    };

    init();

    return () => {
      cancelled = true;
      setReady(false);
      mapRef.current?.remove();
      mapRef.current = null;
      markersRef.current = null;
      iconRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (!ready) return;

    const map = mapRef.current;
    const markers = markersRef.current;
    const icon = iconRef.current;
    if (!map || !markers || !icon) return;

    let cancelled = false;

    const update = async () => {
      const L = (await import('leaflet')).default;
      if (cancelled || !mapRef.current || !markersRef.current || !iconRef.current) return;

      markers.clearLayers();

      locations.forEach(({ lat, lng }, index) => {
        L.marker([lat, lng] as LatLngTuple, { icon })
          .addTo(markers)
          .bindPopup(`Waste pickup #${index + 1}`);
      });

      if (locations.length > 1) {
        map.fitBounds(markers.getBounds(), { padding: [50, 50] });
      } else if (locations.length === 1) {
        map.setView([locations[0].lat, locations[0].lng], 14);
      } else {
        map.setView(DEFAULT_CENTER, 12);
      }
    };

    update();
    return () => {
      cancelled = true;
    };
  }, [locations, ready]);

  return <div ref={containerRef} className="h-full w-full rounded-lg" />;
};

export default MapComponent;
