'use client';

import { useEffect, useState } from 'react';
import { usePathname } from 'next/navigation';

export default function PageTransition({ children }: { children: React.ReactNode }) {
  const pathname = usePathname();
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    setLoading(true);
    const timer = setTimeout(() => setLoading(false), 500);
    return () => clearTimeout(timer);
  }, [pathname]);

  return (
    <>
      {loading && (
        <div className="fixed inset-0 z-[900] flex items-center justify-center bg-green-50/70 backdrop-blur-[2px] pointer-events-none">
          <div className="flex flex-col items-center gap-3">
            <div className="h-10 w-10 rounded-full border-4 border-green-200 border-t-green-600 animate-spin" />
            <p className="text-sm font-medium text-green-700">Loading...</p>
          </div>
        </div>
      )}
      <div className={`transition-opacity duration-300 ${loading ? 'opacity-60' : 'opacity-100'}`}>
        {children}
      </div>
    </>
  );
}
