'use client';
import { useEffect, useRef, useState } from 'react';
import type { LatLngTuple, Map as LeafletMap, FeatureGroup, Icon } from 'leaflet';
import 'leaflet/dist/leaflet.css';

interface MapComponentProps {
  locations: { lat: number; lng: number }[];
}

const DEFAULT_CENTER: LatLngTuple = [12.9716, 77.5946];

type LeafletContainer = HTMLDivElement & { _leaflet_id?: number };

function clearLeafletContainer(container: HTMLElement) {
  const stamped = container as LeafletContainer;
  if (stamped._leaflet_id != null) {
    delete stamped._leaflet_id;
  }
  if (container.childNodes.length > 0) {
    container.replaceChildren();
  }
}

function isMapAlive(map: LeafletMap | null): map is LeafletMap {
  if (!map) return false;
  try {
    return !!map.getContainer()?.isConnected;
  } catch {
    return false;
  }
}

function destroyMap(map: LeafletMap | null, container: HTMLElement | null) {
  if (map) {
    try {
      map.stop();
      map.remove();
    } catch {
      // Map may already be torn down during route transitions
    }
  }
  if (container) {
    clearLeafletContainer(container);
  }
}

const MapComponent: React.FC<MapComponentProps> = ({ locations }) => {
  const mapRef = useRef<LeafletMap | null>(null);
  const markersRef = useRef<FeatureGroup | null>(null);
  const iconRef = useRef<Icon | null>(null);
  const leafletRef = useRef<typeof import('leaflet') | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const aliveRef = useRef(false);
  const initIdRef = useRef(0);
  const [ready, setReady] = useState(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    aliveRef.current = true;
    const initId = ++initIdRef.current;

    const init = async () => {
      const L = await import('leaflet');
      if (!aliveRef.current || initId !== initIdRef.current || !containerRef.current) return;

      const el = containerRef.current;
      destroyMap(mapRef.current, el);
      mapRef.current = null;
      markersRef.current = null;

      if (!aliveRef.current || initId !== initIdRef.current) return;

      leafletRef.current = L;

      iconRef.current = new L.Icon({
        iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
        iconSize: [25, 41],
        iconAnchor: [12, 41],
        popupAnchor: [1, -34],
        shadowSize: [41, 41],
      });

      const map = L.map(el, {
        preferCanvas: true,
        fadeAnimation: false,
        zoomAnimation: false,
      }).setView(DEFAULT_CENTER, 12);

      if (!aliveRef.current || initId !== initIdRef.current) {
        try {
          map.remove();
        } catch {
          // Ignore teardown if init was superseded
        }
        return;
      }

      mapRef.current = map;

      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors',
      }).addTo(map);

      markersRef.current = L.featureGroup().addTo(map);

      map.whenReady(() => {
        if (!aliveRef.current || initId !== initIdRef.current || !isMapAlive(mapRef.current)) return;
        try {
          mapRef.current!.invalidateSize();
        } catch {
          // Ignore resize while tearing down
        }
        setReady(true);
      });
    };

    init();

    return () => {
      aliveRef.current = false;
      initIdRef.current += 1;
      setReady(false);
      const instance = mapRef.current;
      mapRef.current = null;
      markersRef.current = null;
      iconRef.current = null;
      leafletRef.current = null;
      destroyMap(instance, container);
    };
  }, []);

  useEffect(() => {
    if (!ready) return;

    let frame = 0;
    let cancelled = false;

    const updateMarkers = () => {
      if (cancelled || !aliveRef.current) return;

      const map = mapRef.current;
      const markers = markersRef.current;
      const icon = iconRef.current;
      const L = leafletRef.current;
      if (!isMapAlive(map) || !markers || !icon || !L) return;

      try {
        markers.clearLayers();

        locations.forEach(({ lat, lng }, index) => {
          L.marker([lat, lng], { icon })
            .bindPopup(`Waste pickup #${index + 1}`)
            .addTo(markers);
        });

        map.invalidateSize();

        if (locations.length > 1) {
          const bounds = markers.getBounds();
          if (bounds.isValid()) {
            map.fitBounds(bounds, { padding: [50, 50], animate: false });
          }
        } else if (locations.length === 1) {
          map.setView([locations[0].lat, locations[0].lng], 14, { animate: false });
        } else {
          map.setView(DEFAULT_CENTER, 12, { animate: false });
        }
      } catch {
        // Ignore updates while the map is being destroyed or resized
      }
    };

    frame = requestAnimationFrame(updateMarkers);

    return () => {
      cancelled = true;
      if (frame) cancelAnimationFrame(frame);
    };
  }, [locations, ready]);

  return <div ref={containerRef} className="leaflet-contained h-full w-full rounded-lg" />;
};

export default MapComponent;
