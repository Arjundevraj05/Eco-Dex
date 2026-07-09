'use client';
import { useEffect, useRef } from 'react';
import L, { LatLngTuple, Map as LeafletMap } from 'leaflet';
import 'leaflet/dist/leaflet.css';

const customIcon = new L.Icon({
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41],
});

interface MapComponentProps {
  locations: { lat: number; lng: number }[];
}

const DEFAULT_CENTER: LatLngTuple = [12.9716, 77.5946];

const MapComponent: React.FC<MapComponentProps> = ({ locations }) => {
  const mapRef = useRef<LeafletMap | null>(null);
  const markersRef = useRef<L.FeatureGroup | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const map = L.map(containerRef.current).setView(DEFAULT_CENTER, 12);
    mapRef.current = map;

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
    }).addTo(map);

    markersRef.current = L.featureGroup().addTo(map);

    return () => {
      map.remove();
      mapRef.current = null;
      markersRef.current = null;
    };
  }, []);

  useEffect(() => {
    const map = mapRef.current;
    const markers = markersRef.current;
    if (!map || !markers) return;

    markers.clearLayers();

    locations.forEach(({ lat, lng }, index) => {
      const coordinates: LatLngTuple = [lat, lng];
      L.marker(coordinates, { icon: customIcon })
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
  }, [locations]);

  return <div ref={containerRef} className="h-full w-full rounded-lg" />;
};

export default MapComponent;
