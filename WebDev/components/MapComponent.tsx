'use client';
import { useEffect } from 'react';
import L, { LatLngTuple } from 'leaflet';
import 'leaflet/dist/leaflet.css';

const MapComponent = () => {
  useEffect(() => {
    // Set custom icon path
    delete L.Icon.Default.prototype._getIconUrl;
    L.Icon.Default.prototype._getIconUrl = function (name) {
      const iconPath = '/leaflet/images'; // Update this to your custom path
      if (name === 'icon') {
        return `${iconPath}/marker-icon.png`;
      } else if (name === 'shadow') {
        return `${iconPath}/marker-shadow.png`;
      }
      return null;
    };

    // Check if the map is already initialized
    const mapContainer = document.getElementById('map') as HTMLElement;
    if (mapContainer && (mapContainer as any)._leaflet_id) {
      return; // If the map already exists, do nothing
    }

    // Initialize the map if not already initialized
    const map = L.map('map').setView([51.505, -0.09], 13);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
    }).addTo(map);

    // Define ragbot locations with their coordinates
    const ragbotLocations: { name: string; coordinates: LatLngTuple }[] = [
      { name: 'Ragbot 1', coordinates: [51.505, -0.09] },
      { name: 'Ragbot 2', coordinates: [51.515, -0.1] },
      { name: 'Ragbot 3', coordinates: [51.525, -0.08] },
    ];

    // Add markers for each ragbot location
    ragbotLocations.forEach(location => {
      L.marker(location.coordinates)
        .addTo(map)
        .bindPopup(location.name); // Popup shows the ragbot name
    });

    return () => {
      // Clean up the map instance when the component is unmounted
      map.remove();
    };
  }, []);

  return <div id="map" className="h-full w-full" />;
};

export default MapComponent;
