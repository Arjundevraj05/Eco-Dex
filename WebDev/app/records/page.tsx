// app/(root)/records/page.tsx
'use client';

import { useEffect, useState } from 'react';

const RecordsPage = () => {
  const [records, setRecords] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchRecords = async () => {
      try {
        const response = await fetch('/api/users/current'); // Adjust this path if necessary
        if (!response.ok) {
          throw new Error('Failed to fetch records');
        }
        const data = await response.json();
        setRecords(data);
      } catch (err) {
        setError(err.message);
      } finally {
        setLoading(false);
      }
    };

    fetchRecords();
  }, []);

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error}</div>;

  return (
    <div>
      <h1>Records</h1>
      <ul>
        {records.map((record, index) => (
          <li key={index}>
            {JSON.stringify(record)}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default RecordsPage;
